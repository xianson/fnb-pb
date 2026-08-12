using System;
using VRageMath;

namespace IngameScript
{
    partial class Program
    {
        // QTRT = smooth-path tracking. Flies a C1 reference path (CatmullRomSpline) with a
        // pure-pursuit lookahead nose, a lateral PD cross-track loop and a curvature-limited speed
        // schedule, and closes a Stop terminal with a decisive flip-and-burn.
        //
        // Ported from the mod's Autopilot/Control/QtrtController.cs. Two blocks are deliberately absent:
        //   - the HUD/route ETA machinery (CALIB-B/D/F/H/I/J/K). Display-only. EstimateTotalSeconds,
        //     which sets the flight Timeout, is kept and is numerically identical (see there).
        //   - the flight-time asteroid dodge. A PB has no voxel API, so the mod's DodgeProbe would be
        //     null and UpdateDodge would return Vector3D.Zero every tick; the tracked reference is
        //     therefore the bare on-path point, exactly as in the mod with no probe installed.
        public sealed class QtrtController
        {
            public enum TerminalKind { Stop, Flythrough, Spline }

            readonly IShip _ship;
            readonly OrientationController _att;
            readonly CatmullRomSpline _spline;
            readonly TerminalKind _terminal;

            // Sampled speed schedule along the spline (u in [0, SegmentCount]).
            readonly int _nSamp;
            readonly double[] _uSamp;     // parameter at each sample
            readonly double[] _sSamp;     // cumulative arc length at each sample
            readonly double[] _vSamp;     // scheduled speed at each sample
            readonly double[] _kSamp;     // curvature at each sample
            readonly Vector3D[] _tSamp;   // unit travel tangent at each sample
            // IShip.GravityAt on a PB is position-independent (a PB cannot sample the field away from
            // the ship), so the mod's per-sample _gSamp array collapses to one vector. Same numbers,
            // minus 24 B per sample of retained memory.
            readonly Vector3D _gBake;
            double _totalLen;

            // ---- DEFERRED BAKE ----
            // Building the schedule is O(_nSamp) with a spline evaluation per sample; at the
            // 64-waypoint cap that is ~0.7 ms in one run, well past a PB's 0.5 ms budget. So it is
            // chunked across runs. Every ship reading the bake consumes is FROZEN at engage, so the
            // schedule cannot depend on which run a chunk happened to land in -- with the snapshot the
            // chunked bake is bit-identical to the old single-run bake.
            int _bakeStage;               // 0 = sampling, 1 = finish pass pending, 2 = done
            int _bakeI;
            Vector3D _bakePrev;
            bool _bakeHavePrev;
            double _bakeCum;
            readonly double _bkAMax, _bkAtt, _bkEffOmega, _bkALat;
            readonly Vector3D _bkPos, _bkVel;

            public bool BakeDone { get { return _bakeStage >= 2; } }
            public int BakeSamplesLeft
            {
                get { return _bakeStage >= 2 ? 0 : (_nSamp - _bakeI) + _nSamp / 2; }
            }

            double _u;                    // current tracking parameter (monotone-ish)
            Vector3D _brakeAxis; bool _haveBrakeAxis;
            // Latched pro/retrograde nose choice; see the hysteresis block in the nose command.
            int _noseSign; bool _haveNoseSign; double _noseFlipHold;
            // Fraction of available authority the tangential demand must reach IN THE NEW DIRECTION,
            // and how long it must hold there, before the ship is turned around.
            public static double NoseFlipHysteresisFrac = 0.15;
            public static double NoseFlipMinDwellS = 1.0;
            // PARTIAL-FLIP brake state: slew-limited tangential brake accel carried between ticks.
            double _pfBrakeAtan; bool _havePfBrake;
            // Sticky: gravity was significant at SOME point this flight (a planet flight).
            bool _everHadGravity;
            bool _gravDescentHold;        // sticky terminal hover latch (gravity landing)
            bool _termFlipDone;           // sticky: the terminal 180 brake-flip has been performed
            bool _termArrest;             // latched terminal-arrest state
            double _brakeLatchSpeed;      // speed at brake-latch; ceiling so the brake never re-accelerates

            // Committed terminal-brake latch (read/restore).
            public bool TermFlipDone { get { return _termFlipDone; } }
            public double BrakeLatchSpeed { get { return _brakeLatchSpeed; } }
            // Only meaningful with a POSITIVE latch speed: _brakeLatchSpeed is a CEILING.
            public void RestoreBrakeLatch(double latchSpeed)
            {
                if (latchSpeed <= 0.0) return;
                _termFlipDone = true;
                _brakeLatchSpeed = latchSpeed;
            }

            // Latched once the terminal stop-brake is first required; never cleared within a sub-mission.
            bool _stopBrakeLatched;
            // Hysteresis latch for the cross-track position-gain fade (see the note at its use site).
            bool _posFadeArmed;

            public bool IsDone { get; private set; }
            public double Elapsed { get; private set; }
            public double Timeout = 300.0;


            // ---- SHARP-CORNER FEASIBILITY --------------------------------------------
            public static double CornerCentReserve = 0.7;
            public static double CornerWidenCap = 2.0;
            public static double CrossSettleTaus = 4.0;
            public static double ReversalSettleMult = 8.0;
            public static double TurnAxisAlignCos = 0.5;
            public static double LegStraightFrac = 0.25;
            public static double CornerMinTurn = 0.20;
            public static double SoftSplineTolLegFrac = 0.01;
            public static double SoftSplineTolMin = 25.0;
            public static double SoftSplineTolMax = 75.0;
            double _lastCornerU;
            readonly Vector3D[] _cornerPos;
            readonly double[] _cornerTurn;
            readonly double[] _cornerTol;

            // ---- design factors (DIMENSIONLESS; generic across hulls) -----------------
            // Lookahead in attitude time-constants.
            public static double LeadTau = 0.4;
            // Cross-track loop bandwidth as a fraction of the attitude bandwidth.
            public static double CrossLoopSep = 0.5;
            public static double CrossDamping = 1.0;
            // Fraction of aMax the curvature SPEED CAP allots to turning.
            public static double CentReserve = 0.7;
            public static double GyroRateMargin = 0.6;
            // ---- PARTIAL-FLIP corner brake -------------------------------------------
            public static double PartialFlipSlewFrac = 1.0;
            public static double PartialFlipMaxDepth = 0.6;
            public static double PartialFlipFeasMargin = 0.7;
            public static double PartialFlipMaxGravFrac = 0.05;
            // Nose demands under this fraction of drive authority hold the fallback nose instead
            // of steering toward the demand's direction; see the call site.
            public static double NoseDemandFloorFrac = 0.05;
            // ABSOLUTE CEILING ON THAT FLOOR. The floor exists to ignore residual cross-track PD
            // noise, which is small in ABSOLUTE terms -- but expressing it as a fraction of aMax
            // makes it grow with the drive, so the stronger the ship the larger a genuine demand it
            // discards. Measured on the Pickaxe: aMax 197.42 m/s^2 (245 MN on 1.25 Mkg) put the
            // floor at 9.87, above the schedule's real cruise demand of 7.51, so the throttle was
            // pinned to exactly zero and the ship never left the start of a 9 km leg -- gate 1.00,
            // envelope 1.00, thr 0.000. Below ~10 m/s^2 of authority this cap changes nothing.
            public static double NoseDemandFloorAbsMax = 0.5;   // m/s^2
            // Fraction of the main drive the flight PLAN is baked against. Sneak mode sets this to
            // the FLOOR fraction: every speed and brake commit assumes only the quiet drive, so
            // the terminal is always flyable at low signature.
            public static double ThrustFrac = 1.0;
            // Sneak envelope: received signature at an observer scales as thrust/d^2, and the
            // observers sit at the leg's endpoints. Throttle is held to the floor near BOTH ends
            // and released quadratically with distance from the nearest one -- what either
            // endpoint sees stays roughly flat and low while the loud burn happens mid-leg, far
            // from everyone. The schedule stays floor-baked, so the extra mid-leg authority only
            // ever arrives EARLY at speeds the quiet drive can still brake from.
            public static double SneakRampM = 4000.0;
            // Target-sweep rate below which the rate-track branch stays off and the min-time law
            // owns the nose (rad/s). See the phiDot gate.
            public static double OmegaTrackMinSweep = 0.002;
            // Minimum path curvature (1/m) for the rate-track branch to own the nose. Speed-
            // INDEPENDENT, unlike the phiDot gate. ~1e-4 = a 10 km radius; real corners far
            // exceed it, straight-leg numerical noise (~1e-5) never reaches it.
            public static double CurvatureTrackMin = 1.0e-4;
            // Hard ceiling on scheduled cruise speed (m/s). 0 = UNCAPPED (default): the target
            // server's world limit is 50k and a low cap would waste it. This is a per-world SAFETY
            // knob, not a shipped limit -- set `maxspeed` on a world where a long leg would
            // otherwise accelerate past the hull's flip-brake capability and run away. The
            // speed-agnostic RunawayGuard below is the real containment. See BuildSchedule.
            public static double MaxCruiseSpeed = 0.0;
            // Seed for uncapped samples; see BuildSchedule. Numeric guard only.
            public static double VelCeiling = 1.0e6;   // no cap: the schedule flies at the hull's own flip-brake-feasible speed
            // Flip coast reserved before a brake, in units of the bang-bang 180 slew time.
            public static double FlipReserveFactor = 2.0;
            public static double TermFlipFactor = 1.4;
            public static double StraightFlipFactor = 1.0;
            // Brake-throttle reserve: aim the v=0 point this fraction short of the geometric stop.
            public static double BrakeMargin = 0.92;
            public static double StraightBrakeMargin = 0.94;
            // Thrust align gate: main drive fires scaled by cos(headingErr), hard-cut beyond this.
            public static double AlignCutoffRad = 60.0 * Math.PI / 180.0;
            // Brake-side cutoff: the retro burn must not light while it still moves its own aim point.
            public static double BrakeAlignCutoffRad = 15.0 * Math.PI / 180.0;

            // ---- FLIP BANG-BANG HANDOFF ----------------------------------------------
            public static bool FlipBangBangHandoff = true;
            public static double FlipHandoffSweepMax = 0.02;
            public static double FlipEnterErrRad = 40.0 * Math.PI / 180.0;
            public static double FlipExitErrRad = 12.0 * Math.PI / 180.0;
            public static double FlipRelatchRad = 20.0 * Math.PI / 180.0;
            // Latched flip aim point (world unit vector) + state.
            Vector3D _flipNose; bool _flipLatched;

            public static bool ProgradeWeakAlignNarrow = true;
            public static double ProgradeAlignCutoffWeakRad = 30.0 * Math.PI / 180.0;

            // ---- TERMINAL-ARREST NOSE BOUND -------------------------------------------
            public static double TerminalNoseCapRad = 0.5 * AlignCutoffRad;
            public static bool TermFlipContinuous = true;
            public static double TermFlipSpanRad = Math.PI;
            public static bool ArrestLatFloor = true;
            public static bool GravStallRelease = true;

            // ---- FROM-REST LAUNCH ALIGN GATE ------------------------------------------
            public static double LaunchAlignSpeed = 25.0;
            public static double LaunchAlignRad = 0.20;
            public static double LaunchAlignMaxGravFrac = 0.05;

            // ---- TACK: drive-signature obfuscation ------------------------------------
            // Spectrum's drive emission is a directional lobe (3 deg half-angle, gain 4) welded to a
            // cardinal hull face, so range is 4x inside it. A straight flip-and-burn therefore aims a
            // 4x searchlight down the track: at the origin while accelerating, at the destination
            // while braking. Holding the drive off the track keeps the lobe off both.
            // 0 = off. 30 deg is the knee: 2.65x less range for 15.5% more propellant.
            public static double TackAngleRad = 0.0;
            // Coning period. The cant AZIMUTH rolls at a constant rate rather than flipping sign, so
            // the lobe never sweeps back through the track, and lateral velocity is mean-zero by
            // construction instead of needing a cancellation schedule.
            public static double TackConeSeconds = 60.0;
            // 0 = full cone, 1 = fixed azimuth. THIS IS AN EXPOSURE TRADE, NOT JUST AN EFFICIENCY ONE.
            // A cone keeps lateral velocity mean-zero for free, but the swept lobe lights ~6x the
            // solid angle a fixed pencil does -- it protects the on-track observer by broadcasting to
            // an annulus a straight burn never touches. A fixed azimuth stays a pencil and also bows
            // the track (which is what fools an observer who INTEGRATES the picture rather than
            // sampling it), paid for in lateral velocity the cross-track loop has to fight.
            public static double TackBias = 0.0;
            // rad/s the applied cant is allowed to move, so it eases in and out of the gates.
            public static double TackFadeRate = 0.3;

            Vector3D _tackU, _tackW;
            bool _haveTackFrame;
            double _tackPhase;
            double _tackCur;      // cant actually applied this tick, after the terminal fade

            public static double TackCos { get { return Math.Cos(TackAngleRad); } }

            // ---- HOT-START schedule seed ----------------------------------------------
            public static double HotStartPosEps = 5.0;

            // Handoff: hand to Fine once near the terminal and slow enough for dampeners.
            public static double HandoffSpeed = 25.0;

            // Gravity-descent terminal hover-hold.
            public static double GravityHoverRadius = 70.0;
            public static double GravityHoverArriveD = 30.0;
            public static double GravityHoverExitSpd = 5.0;
            public static double GravityHoverPosGain = 0.5;
            public static double GravityHoverVelGain = 1.4;
            public static double GravityDescentVelGate = 5.0;
            public static double HoverRcsHoldFrac = 0.9;

            double _perpPosGain, _perpVelGain;
            double _tFlip;           // 180-deg flip coast time (corner/curve reserve)
            double _tFlipStraight;   // tighter reserve used ONLY on a straightRun terminal stop
            double _bmStraight;      // weakness-blended straight brake margin
            double _gyroWeakness;    // 0 = bandwidth meets the rate cap, 1 = no attitude authority
            double _alphaFlip;       // MaxTorque/I (rad/s^2) used for the flip slew
            double _slewSpan;        // SlewTimeLocal(TermFlipSpanRad, _alphaFlip)

            // Per-corner schedule-cap arc positions and feasible speeds.
            double[] _cornerS = new double[0];
            double[] _cornerV = new double[0];

            // Max speed from which the ship can still stop within d, accounting for the 180 flip coast.
            double BrakeFeasibleSpeed(double d, double aMax, double tFlip)
            {
                if (d <= 0.0) return 0.0;
                double disc = tFlip * tFlip + 2.0 * d / aMax;
                return (-tFlip + Math.Sqrt(disc)) * aMax;
            }

            // Runaway guard: receding-from-target dwell + latch. See Update.
            public static double RunawaySeconds = 6.0;
            public static double RunawayMinDistM = 3000.0;
            double _recedeS;
            public bool RunawayBraking;

            // Telemetry/labels.
            public string DbgPhase = "init";
            public double DbgVSched, DbgAlignErr;
            // The three factors of the throttle write. A zero throttle is ambiguous without them:
            // the demand, the alignment/interlock gate and the sneak envelope multiply, and any one
            // of them at zero looks the same from outside.
            public double DbgThr, DbgThrGate, DbgEnvFrac, DbgATan, DbgAMax;
            public int DbgStraightRun;
            // PRODUCTION state, not diagnostic: cross-track offset (m) and along-track speed (m/s).
            public double DbgCross;
            public double AlongTrackSpeed;
            // Brake-execution diagnostic: the retrograde tangential demand actually applied, the cap the
            // perpendicular demand leaves for it, and the perpendicular demand magnitude. If aPerp
            // squeezes aTanCap so aTan can't reach the commanded -aMax, the nose never flips.
            public double DbgAtan, DbgAtanCap, DbgAperp;
            // Nose-demand diagnostic. Separates "the ship is slow" from "the target is moving":
            // sweep is the commanded nose's own angular rate (deg/s), perpFrac how much of aDesired
            // came from cross-track excess rather than the tangent, and floor whether the demand
            // fell below NoseDemandFloor (a hard switch to the pro/retrograde fallback).
            public double DbgNoseSweep, DbgPerpFrac;
            // Which attitude law ran, the max-hold alpha it sized its braking curve with, and the
            // resulting rate ceiling. An inflated alpha here IS an overshoot: it tells the law it
            // can stop from a rate it cannot.
            public string DbgLaw = "none";
            public double DbgAlphaObs, DbgRateCeil;
            public int DbgNoseFloor;
            Vector3D _dbgPrevNose; bool _dbgHavePrevNose;
            // Flip hand-off diagnostic: lead-point curvature, the OmegaTrack rate-track feedforward
            // magnitude (-1 = null/min-time), and the gyro command magnitude the OrientationController
            // actually produced this tick. If the nose is 180 off, OmegaTrack is null, yet GyroCmd is 0,
            // the command is being dropped downstream; if GyroCmd is nonzero, the plant/gyro is.
            public double DbgKLead, DbgOmegaTrackMag, DbgGyroCmd;

            public QtrtController(IShip ship, OrientationController att,
                                  CatmullRomSpline spline, TerminalKind terminal,
                                  Vector3D[] cornerPos, double[] cornerTurn, double[] cornerTol)
            {
                _ship = ship;
                _att = att;
                _spline = spline;
                _terminal = terminal;
                _cornerPos = cornerPos;
                _cornerTurn = cornerTurn;
                _cornerTol = cornerTol;

                double wAtt = AttitudeBandwidth();
                double wCross = CrossLoopSep * wAtt;
                _perpPosGain = wCross * wCross;
                _perpVelGain = 2.0 * CrossDamping * wCross;
                // The REAL world limit, not the nominal constant.
                _att.CommandRateCap = _ship.GyroRateCap > 1e-6
                    ? _ship.GyroRateCap
                    : OrientationController.CommandedRateCapBase;

                // ---- allocate; the sampling and schedule build run from BakeStep ----
                int segs = Math.Max(1, _spline.SegmentCount);
                _nSamp = segs * SamplesPerSegment + 1;
                _uSamp = new double[_nSamp];
                _sSamp = new double[_nSamp];
                _vSamp = new double[_nSamp];
                _kSamp = new double[_nSamp];
                _tSamp = new Vector3D[_nSamp];

                // Engage snapshot. wAtt is reused rather than re-read: MaxTorque is re-measured every
                // run, so a live re-read partway through a chunked bake would shift the schedule.
                double aMax = _ship.MaxForwardThrust * _ship.Body.InvMass * TackCos * ThrustFrac;
                if (aMax < 1e-6) aMax = 1e-6;
                _bkAMax = aMax;
                _bkAtt = wAtt;
                _bkEffOmega = EffGyroRateCapWith(wAtt);
                _bkALat = _ship.MaxLateralThrust * _ship.Body.InvMass;
                _bkPos = _ship.Body.Position;
                _bkVel = _ship.Body.LinearVelocity;
                _gBake = _ship.GravityAt(_bkPos);
                _u = 0.0;
            }

            // Advance the schedule build by at most `budget` spline samples. Returns the sample-equivalent
            // work consumed, so a caller can share one run's budget across several sub-missions.
            public int BakeStep(int budget)
            {
                if (_bakeStage >= 2) return 0;
                bool unlimited = budget <= 0;   // 0 = bake it all now
                int used = 0;
                if (_bakeStage == 0)
                {
                    int segs = Math.Max(1, _spline.SegmentCount);
                    int end = unlimited ? _nSamp : _bakeI + budget;
                    if (end > _nSamp) end = _nSamp;
                    for (int i = _bakeI; i < end; i++)
                    {
                        double u = (i / (double)(_nSamp - 1)) * segs;
                        // One EvalSegment per sample instead of two: Position and CurvatureAt derived
                        // the same (seg, t) and both evaluated it.
                        Vector3D tan; double kRaw;
                        Vector3D p = _spline.Sample(u, out tan, out kRaw);
                        if (_bakeHavePrev) _bakeCum += (p - _bakePrev).Length();
                        _bakePrev = p; _bakeHavePrev = true;
                        _uSamp[i] = u;
                        _sSamp[i] = _bakeCum;
                        if (double.IsNaN(kRaw) || kRaw < 0.0) kRaw = 0.0;
                        _kSamp[i] = kRaw;
                        _tSamp[i] = tan.LengthSquared() > 1e-12 ? Vector3D.Normalize(tan) : Vector3D.UnitZ;
                    }
                    used = end - _bakeI;
                    _bakeI = end;
                    if (_bakeI < _nSamp) return used;
                    _totalLen = _bakeCum;
                    _bakeStage = 1;
                }
                // The finish pass (phantom guard, corner caps, schedule passes) measures at roughly half
                // a full sampling sweep, so it is charged _nSamp/2 and given its own run when what is
                // left of this run's budget will not cover it.
                if (!unlimited && used > 0 && budget - used < _nSamp / 2) return used;
                BakeFinish();
                _bakeStage = 2;
                return used + _nSamp / 2;
            }

            // Everything between the sampling sweep and the finished schedule. Reads only the frozen
            // engage snapshot, never live ship state.
            void BakeFinish()
            {
                double aMax = _bkAMax;

                // ---- PHANTOM-ENDPOINT CURVATURE GUARD ---------------------------------
                // The grid lands exactly on the two Catmull-Rom phantom-endpoint knots, where curvature
                // can read infinite; clamp both to the interior peak.
                double kInterior = 0.0;
                for (int i = 1; i < _nSamp - 1; i++)
                    if (!double.IsInfinity(_kSamp[i]) && _kSamp[i] > kInterior) kInterior = _kSamp[i];
                for (int i = 0; i < _nSamp; i++)
                    if (double.IsInfinity(_kSamp[i]) || _kSamp[i] > kInterior) _kSamp[i] = kInterior;

                // Flip coast reserved at the terminal brake = the BANG-BANG 180 slew time.
                double alphaFlip = _bkAtt * _bkAtt;
                // GYRO-WEAKNESS adaptive reserve: a weak hull flips slowly and needs the full factor.
                double omCmd = OrientationController.CommandedRateCapBase;
                if (omCmd > OrientationController.GyroHardwareRateCap) omCmd = OrientationController.GyroHardwareRateCap;
                double weakness = omCmd > 1e-6 ? Math.Max(0.0, 1.0 - _bkAtt / omCmd) : 0.0;
                double termFactor = TermFlipFactor + (FlipReserveFactor - TermFlipFactor) * Math.Min(1.0, weakness / 0.3);
                _tFlip = termFactor * SlewTimeWith(Math.PI, alphaFlip, _bkEffOmega);
                double straightFactor = StraightFlipFactor + (FlipReserveFactor - StraightFlipFactor) * Math.Min(1.0, weakness / 0.3);
                if (straightFactor > termFactor) straightFactor = termFactor;
                _tFlipStraight = straightFactor * SlewTimeWith(Math.PI, alphaFlip, _bkEffOmega);
                _alphaFlip = alphaFlip;
                _slewSpan = SlewTimeWith(TermFlipSpanRad, alphaFlip, _bkEffOmega);
                _bmStraight = StraightBrakeMargin + (BrakeMargin - StraightBrakeMargin) * Math.Min(1.0, weakness / 0.3);
                if (_bmStraight < BrakeMargin) _bmStraight = BrakeMargin;
                if (_bmStraight > StraightBrakeMargin) _bmStraight = StraightBrakeMargin;
                _gyroWeakness = weakness;

                // ---- SHARP-CORNER feasible-speed caps ---------------------------------
                if (_cornerPos != null && _cornerTurn != null
                    && _cornerTol != null && _cornerPos.Length == _cornerTurn.Length
                    && _cornerPos.Length == _cornerTol.Length)
                {
                    // PASS 1: each genuine corner's arc position, R_tol corridor speed and peak curvature.
                    var sList = new System.Collections.Generic.List<double>();
                    var vRtolList = new System.Collections.Generic.List<double>();
                    var kPeakList = new System.Collections.Generic.List<double>();
                    var binList = new System.Collections.Generic.List<Vector3D>();
                    for (int c = 0; c < _cornerPos.Length; c++)
                    {
                        double t = _cornerTurn[c];
                        double tol = _cornerTol[c];
                        if (t <= 1e-3 || tol <= 0.0 || double.IsInfinity(tol) || double.IsNaN(tol))
                            continue;
                        double cosHalf = Math.Cos(0.5 * t);
                        double denom = 1.0 - cosHalf;
                        if (denom < 1e-6) continue;
                        double rTol = tol * cosHalf / denom;
                        if (rTol < 1e-3) continue;
                        double vCorner = Math.Sqrt(CornerCentReserve * aMax * rTol);
                        double uC = _spline.ClosestU(_cornerPos[c], 0.0);
                        if (uC > _lastCornerU) _lastCornerU = uC;
                        double sC = InterpAt(uC, _sSamp);
                        Vector3D bin; _spline.CurvatureWithAxis(uC, out bin);
                        sList.Add(sC);
                        vRtolList.Add(vCorner);
                        kPeakList.Add(PeakCurvatureNear(sC));
                        binList.Add(bin);
                    }
                    _cornerS = sList.ToArray();
                    _cornerV = vRtolList.ToArray();

                    // PASS 2: widen by the spline's ACTUAL peak curvature where the corner is isolated.
                    if (_cornerS.Length > 0)
                    {
                        double wCrossLoop = CrossLoopSep * _bkAtt;
                        for (int c = 0; c < _cornerS.Length; c++)
                        {
                            double kPeak = kPeakList[c];
                            if (kPeak <= 1e-9) continue;
                            double vActual = Math.Sqrt(CornerCentReserve * aMax / kPeak);
                            double vRtol = _cornerV[c];
                            if (vActual <= vRtol) continue;
                            double sPrev = (c > 0) ? _cornerS[c - 1] : 0.0;
                            double sNext = (c < _cornerS.Length - 1) ? _cornerS[c + 1] : _totalLen;
                            double legMin = Math.Min(_cornerS[c] - sPrev, sNext - _cornerS[c]);
                            // TURN-PLANE CHANGE: the cheap widen is safe only for a planar neighbourhood.
                            double minAlign = 1.0;
                            Vector3D binC = binList[c];
                            if (binC.LengthSquared() > 1e-12)
                            {
                                if (c > 0 && binList[c - 1].LengthSquared() > 1e-12)
                                    minAlign = Math.Min(minAlign, Vector3D.Dot(binC, binList[c - 1]));
                                if (c < _cornerS.Length - 1 && binList[c + 1].LengthSquared() > 1e-12)
                                    minAlign = Math.Min(minAlign, Vector3D.Dot(binC, binList[c + 1]));
                            }
                            bool planeChange = minAlign < TurnAxisAlignCos;
                            double kMidPrev = CurvatureAtArc(0.5 * (sPrev + _cornerS[c]));
                            double kMidNext = CurvatureAtArc(0.5 * (_cornerS[c] + sNext));
                            double kLegFloor = Math.Max(kMidPrev, kMidNext);
                            bool neverStraightens = kPeak > 1e-12 && kLegFloor > LegStraightFrac * kPeak;
                            double settleMult = (planeChange || neverStraightens) ? ReversalSettleMult : 1.0;
                            double settle = wCrossLoop > 1e-6 ? settleMult * vActual * CrossSettleTaus / wCrossLoop : double.MaxValue;
                            double iso = settle > 1e-6 ? (legMin / settle - 1.0) : 1.0;
                            if (iso <= 0.0) continue;
                            if (iso > 1.0) iso = 1.0;
                            double vWide = vRtol + (vActual - vRtol) * iso;
                            if (vWide > CornerWidenCap * vRtol) vWide = CornerWidenCap * vRtol;
                            double vAuthCap = AuthorityWidenCeiling(kPeak, aMax);
                            if (vAuthCap < vRtol) vAuthCap = vRtol;
                            if (vWide > vAuthCap) vWide = vAuthCap;
                            if (vWide > vRtol) _cornerV[c] = vWide;
                        }
                    }
                }

                BuildSchedule(aMax);
            }

            // Schedule resolution. The mod uses 48; a PB pays for this in both instructions (the
            // constructor evaluates the spline _nSamp times) and in the per-tick RuntimeBrakeCeiling
            // walk, so it stays tunable. Lowering it CHANGES the flown path. 48 = mod-identical.
            public static int SamplesPerSegment = 48;

            // Apply the per-corner R_tol caps to the pointwise schedule BEFORE the dynamic passes.
            void ApplyCornerCaps()
            {
                if (_cornerS.Length == 0) return;
                double win = _nSamp > 1 ? (_totalLen / (_nSamp - 1)) * 1.5 : 1.0;
                for (int c = 0; c < _cornerS.Length; c++)
                {
                    double sC = _cornerS[c];
                    double vC = _cornerV[c];
                    int lo, hi;
                    WindowRange(sC, win, out lo, out hi);
                    for (int i = lo; i < hi; i++)
                        if (Math.Abs(_sSamp[i] - sC) <= win && _vSamp[i] > vC)
                            _vSamp[i] = vC;
                }
            }

            // Peak sampled curvature within the corner window around arc position sC.
            double PeakCurvatureNear(double sC)
            {
                double win = _nSamp > 1 ? (_totalLen / (_nSamp - 1)) * 1.5 : 1.0;
                double kPeak = 0.0;
                int lo, hi;
                WindowRange(sC, win, out lo, out hi);
                for (int i = lo; i < hi; i++)
                    if (Math.Abs(_sSamp[i] - sC) <= win && _kSamp[i] > kPeak) kPeak = _kSamp[i];
                return kPeak;
            }

            // Index range [lo, hi) that is a SUPERSET of { i : |_sSamp[i] - sC| <= win }. _sSamp is
            // monotone non-decreasing (it accumulates chord lengths), so the hit set is contiguous and
            // binary-searchable. The bounds are padded by a few ulps and the caller still applies the
            // exact predicate, so the result is identical to a full linear scan -- this removes the
            // bake's only O(corners * samples) term without touching a single comparison outcome.
            void WindowRange(double sC, double win, out int lo, out int hi)
            {
                double pad = Math.Abs(sC) * 1e-12 + Math.Abs(win) * 1e-12 + 1e-9;
                lo = LowerBound(sC - win - pad);
                hi = LowerBound(sC + win + pad);
                while (hi < _nSamp && _sSamp[hi] <= sC + win + pad) hi++;
                if (lo < 0) lo = 0;
                if (hi > _nSamp) hi = _nSamp;
            }

            // First index with _sSamp[i] >= value, or _nSamp.
            int LowerBound(double value)
            {
                int lo = 0, hi = _nSamp;
                while (lo < hi)
                {
                    int mid = (lo + hi) >> 1;
                    if (_sSamp[mid] < value) lo = mid + 1; else hi = mid;
                }
                return lo;
            }

            // Sampled curvature at arc position s (nearest sample).
            //
            // _sSamp is monotone non-decreasing, so |_sSamp[i] - s| is unimodal and the minimiser is
            // one of the two samples bracketing s. The linear scan's `d < bd` tie-break keeps the
            // LOWEST index among equal distances, so on a tie the bracket index is walked back to the
            // first occurrence of its value -- which reproduces the scan exactly, not approximately.
            double CurvatureAtArc(double s)
            {
                if (_nSamp <= 0) return 0.0;
                int hi = LowerBound(s);                    // first index with _sSamp >= s
                if (hi <= 0) return _kSamp[0];
                if (hi >= _nSamp) return _kSamp[FirstOfValue(_nSamp - 1)];
                int lo = FirstOfValue(hi - 1);
                double dLo = s - _sSamp[lo];
                double dHi = _sSamp[hi] - s;
                return dLo <= dHi ? _kSamp[lo] : _kSamp[hi];
            }

            // Lowest index sharing _sSamp[i]'s value (duplicates arise where a segment has zero length).
            int FirstOfValue(int i)
            {
                double v = _sSamp[i];
                int j = LowerBound(v);
                return j < i ? j : i;
            }

            public static double CornerSweepMargin = 0.30;
            public static double CornerLatMargin = 1.0;

            // The MAX corner speed this hull can actually hold through kPeak.
            double AuthorityWidenCeiling(double kPeak, double aMax)
            {
                if (kPeak <= 1e-9) return double.MaxValue;
                double vSweep = CornerSweepMargin * _bkEffOmega / kPeak;
                double aLat = _bkALat;
                double vLat = Math.Sqrt(Math.Max(0.0, CornerLatMargin * (aLat + CornerCentReserve * aMax) / kPeak));
                return Math.Min(vSweep, vLat);
            }

            public static int GravBakeMode = 1;

            void BuildSchedule(double aMax)
            {
                const double VFLOOR = 8.0;     // never schedule below this (avoid creep stalls)

                // Sneak: per-sample drive authority, quiet at both endpoints and full mid-leg
                // (see SneakRampM). Baking the envelope into BOTH passes is what makes the
                // profile low-LOUD-low instead of quiet-everywhere: the accel pass may climb
                // hard mid-leg, and the brake pass guarantees the run-in to the endpoint is
                // flyable on the quiet floor alone. aMax arrives floor-scaled; recover full.
                double[] envA = null;
                if (ThrustFrac < 1.0 && ThrustFrac > 1e-6)
                {
                    double full = aMax / ThrustFrac;
                    envA = new double[_nSamp];
                    for (int i = 0; i < _nSamp; i++)
                    {
                        double dNear = Math.Min(_sSamp[i], Math.Max(0.0, _totalLen - _sSamp[i]));
                        double x = SneakRampM > 1.0 ? dNear / SneakRampM : 1.0;
                        if (x > 1.0) x = 1.0;
                        envA[i] = full * (ThrustFrac + (1.0 - ThrustFrac) * x * x);
                    }
                }

                // NO PRE-CAP ON CRUISE SPEED. sqrt(aMax * L) is the peak of a bang-bang run over
                // the leg -- a CONSEQUENCE of the accel and brake passes below, not a constraint
                // they need. The backward pass runs from v=0 at the terminal and already
                // guarantees the ship can stop from every sample, which is the only property this
                // cap ever defended; the forward pass then peaks at sqrt(aMax * L) on its own.
                //
                // Imposing it up front bought nothing when aMax was right and was actively
                // destructive when it was not: one understated scalar became a hard speed limit
                // for the entire leg, which no amount of correct thrust downstream could recover.
                // Measured: 145 m/s cruise on a 70 km leg from a hull with ~100 m/s^2 of drive.
                // Let the passes fly it.
                //
                // VelCeiling is a NUMERIC sentinel for samples with no curvature -- not a physical
                // limit. The passes clamp it on the first sweep; a finite value simply keeps v*v
                // out of overflow.
                double vCruiseMax = VelCeiling;
                // HARD CRUISE CEILING. Without it a long leg plans to accelerate to
                // sqrt(a*L) -- 400+ m/s on a 320 km leg -- and a slow-flip hull cannot turn
                // retrograde to brake from that: it sails past the target and runs away (observed
                // 221 km -> 322 km, receding at 727 m/s on a HighSpeed-mod world where physical
                // velocity clamps its REPORT to 300 but the ship truly moves faster). Cap the
                // schedule at a speed the hull can reliably flip-and-brake from. 0 = uncapped.
                if (MaxCruiseSpeed > 1.0 && vCruiseMax > MaxCruiseSpeed) vCruiseMax = MaxCruiseSpeed;
                // 1) pointwise curvature cap, clamped to the cruise ceiling.
                double omegaMax = _bkEffOmega;
                for (int i = 0; i < _nSamp; i++)
                {
                    double k = _kSamp[i];
                    double vCent = k > 1e-9 ? Math.Sqrt(CentReserve * aMax / k) : vCruiseMax;
                    double vGyro = k > 1e-9 ? GyroRateMargin * omegaMax / k : vCruiseMax;
                    double vCap = Math.Min(vCent, vGyro);
                    if (vCap > vCruiseMax) vCap = vCruiseMax;
                    _vSamp[i] = vCap;
                }
                // 1b) SHARP-CORNER feasible-speed caps (R_tol).
                ApplyCornerCaps();

                // 2) endpoint boundary condition: Stop -> 0 at the end.
                if (_terminal == TerminalKind.Stop) _vSamp[_nSamp - 1] = 0.0;

                // 2b) signed per-sample gravity for the accel/brake passes.
                bool anyGrav = _nSamp > 0 && _gBake.LengthSquared() > 0.0;
                double[] gAlong = null, aDrive = null;
                if (anyGrav && GravBakeMode > 0)
                {
                    gAlong = new double[_nSamp];
                    aDrive = new double[_nSamp];
                    for (int i = 0; i < _nSamp; i++)
                    {
                        Vector3D g = _gBake;
                        double ga = Vector3D.Dot(g, _tSamp[i]);
                        gAlong[i] = ga;
                        if (GravBakeMode >= 2)
                        {
                            double gp2 = g.LengthSquared() - ga * ga;
                            if (gp2 < 0.0) gp2 = 0.0;
                            double d2 = aMax * aMax - gp2;
                            aDrive[i] = d2 > 0.0 ? Math.Sqrt(d2) : 0.05 * aMax;
                        }
                        else aDrive[i] = aMax;
                    }
                }

                // 3) backward (brake) pass with CURVATURE-COUPLED tangential budget.
                for (int i = _nSamp - 2; i >= 0; i--)
                {
                    double ds = _sSamp[i + 1] - _sSamp[i];
                    double vNext = _vSamp[i + 1];
                    double aCentNext = _kSamp[i + 1] * vNext * vNext;
                    double aDr = envA != null ? envA[i + 1] : (aDrive != null ? aDrive[i + 1] : aMax);
                    double aTanAvail = Math.Sqrt(Math.Max(0.0, aDr * aDr - aCentNext * aCentNext));
                    if (aTanAvail < 0.05 * aMax) aTanAvail = 0.05 * aMax;
                    if (gAlong != null)
                    {
                        // gravity that aids forward travel (descent) opposes the brake.
                        aTanAvail -= gAlong[i + 1];
                        if (aTanAvail < 0.05 * aMax) aTanAvail = 0.05 * aMax;
                    }
                    double vb = Math.Sqrt(vNext * vNext + 2.0 * aTanAvail * ds);
                    if (_vSamp[i] > vb) _vSamp[i] = vb;
                }
                // 4) forward (accel) pass. HOT-START: seed v[0] from the along-track speed when the
                // spline starts at the ship, so adding a waypoint mid-cruise does not read as a rest
                // start. This is the bake's ONLY live-kinematic input, so it reads the engage snapshot
                // -- otherwise a chunked bake would seed from wherever the ship drifted to meanwhile.
                double vStartSeed = VFLOOR;
                if (_nSamp > 0 && _spline.ControlPoints.Length > 0)
                {
                    Vector3D shipPos = _bkPos;
                    Vector3D splineStart = _spline.ControlPoints[0];
                    if ((splineStart - shipPos).LengthSquared() < HotStartPosEps * HotStartPosEps)
                    {
                        double vAlong0 = Vector3D.Dot(_bkVel, _tSamp[0]);
                        if (vAlong0 > vStartSeed) vStartSeed = vAlong0;
                    }
                }
                // SET _vSamp[0] from the seed rather than min-ing it: sample 0's curvature cap is a
                // phantom-knot artefact, not a real limit.
                double vBrakeCap0 = _nSamp > 1
                    ? Math.Sqrt(_vSamp[1] * _vSamp[1] + 2.0 * aMax * (_sSamp[1] - _sSamp[0]))
                    : vStartSeed;
                _vSamp[0] = Math.Min(vStartSeed, vBrakeCap0);
                for (int i = 1; i < _nSamp; i++)
                {
                    double ds = _sSamp[i] - _sSamp[i - 1];
                    double aAcc = envA != null ? envA[i] : aMax;
                    if (aDrive != null && envA == null)
                    {
                        aAcc = aDrive[i] + gAlong[i];
                        if (aAcc < 0.05 * aMax) aAcc = 0.05 * aMax;
                    }
                    double vf = Math.Sqrt(_vSamp[i - 1] * _vSamp[i - 1] + 2.0 * aAcc * ds);
                    if (_vSamp[i] > vf) _vSamp[i] = vf;
                }
                // 5) floor (except the true Stop terminal sample).
                for (int i = 0; i < _nSamp; i++)
                    if (_vSamp[i] < VFLOOR && !(_terminal == TerminalKind.Stop && i == _nSamp - 1))
                        _vSamp[i] = VFLOOR;
            }

            double AttitudeBandwidth()
            {
                Vector3D I = _ship.Body.InertiaBody;
                double iMax = Math.Max(I.X, Math.Max(I.Y, I.Z));
                if (iMax < 1.0) iMax = 1.0;
                double alpha = _ship.MaxTorque / iMax;
                return alpha > 1e-9 ? Math.Sqrt(alpha) : 1e-3;
            }

            public CatmullRomSpline Spline { get { return _spline; } }
            public double CurrentU { get { return _u; } }

            // Fuel brake-reserve hooks, set by the host fuel guard.
            public bool FuelCoast;
            public bool FuelForceBrake;

            // v_floor used by the schedule (BuildSchedule.VFLOOR).
            const double EtaVFloor = 8.0;

            // Gravity time stretch, TIMEOUT path only (this is the mod's EtaGravTimeMult).
            public static double EtaGravTimeFactor = 4.0;
            public static double EtaGravStretchCap = 0.20;

            double _gravShareCache = -1.0;
            double MeanAlongGravShare(double aMax)
            {
                if (_gravShareCache >= 0.0) return _gravShareCache;
                Vector3D g = _ship.Gravity;
                double gmag = g.Length();
                // DO NOT cache a zero-gravity read.
                if (gmag < 1e-9 || aMax <= 1e-9 || _nSamp < 2) return 0.0;
                Vector3D gHat = g / gmag;
                double gShare = gmag / aMax;
                double wSum = 0.0, accum = 0.0;
                for (int i = 1; i < _nSamp; i++)
                {
                    double ds = _sSamp[i] - _sSamp[i - 1];
                    if (ds <= 0.0) continue;
                    Vector3D tan = _tSamp[i];
                    if (tan.LengthSquared() < 1e-12) continue;
                    double along = Math.Abs(Vector3D.Dot(gHat, tan));
                    accum += along * ds;
                    wSum += ds;
                }
                _gravShareCache = (wSum > 1e-9) ? (accum / wSum) * gShare : 0.0;
                return _gravShareCache;
            }

            double EtaGravTimeMult(double aMax)
            {
                double stretch = EtaGravTimeFactor * MeanAlongGravShare(aMax);
                if (stretch > EtaGravStretchCap) stretch = EtaGravStretchCap;
                return 1.0 + stretch;
            }

            // Full traversal time from a v=0 start. Sets the sub's flight Timeout, so it is kept.
            //
            // NUMERICALLY IDENTICAL to the mod's EstimateTotalSeconds = TotalSecondsOver(_vSamp, aMax,
            // false): with stopCap false the capped branch never runs, so t == tPlain and the mod's
            // final `t = tPlain + EtaFlipCharge(t - tPlain, _tFlip)` reduces to t + _tFlip on a Stop.
            // The mod's EtaScheduleFactor() is 1.0 either way because EtaCurveFactor == 1.00.
            public double EstimateTotalSeconds(double aMax)
            {
                if (_vSamp == null || _nSamp < 2) return 0.0;
                double t = 0.0;
                for (int i = 1; i < _nSamp; i++)
                {
                    double ds = _sSamp[i] - _sSamp[i - 1];
                    if (ds <= 0.0) continue;
                    double vAvg = 0.5 * (Math.Max(_vSamp[i - 1], EtaVFloor) + Math.Max(_vSamp[i], EtaVFloor));
                    t += ds / vAvg;
                }
                t *= EtaGravTimeMult(aMax);
                if (_terminal == TerminalKind.Stop) t += _tFlip;
                return t;
            }

            // APPROXIMATE remaining time, for the status readout only. The mod's closed-loop CALIB-H/I/K
            // march is not ported; this is the same schedule integral run from the live position, so it
            // reads optimistic near a terminal brake. Nothing in the flight path consumes it.
            public double EstimateSecondsRemaining(double aMax)
            {
                if (IsDone || _vSamp == null || _nSamp < 2) return 0.0;
                double segs = Math.Max(1, _spline.SegmentCount);
                double uEta = _spline.ClosestU(_ship.Body.Position, _u);
                if (uEta > segs) uEta = segs;
                if (uEta < 0.0) uEta = 0.0;
                double f0 = uEta / segs * (_nSamp - 1);
                int i0 = (int)Math.Floor(f0);
                if (i0 < 0) i0 = 0;
                double t = 0.0;
                for (int i = i0 + 1; i < _nSamp; i++)
                {
                    double ds = _sSamp[i] - _sSamp[i - 1];
                    if (ds <= 0.0) continue;
                    double vAvg = 0.5 * (Math.Max(_vSamp[i - 1], EtaVFloor) + Math.Max(_vSamp[i], EtaVFloor));
                    t += ds / vAvg;
                }
                t *= EtaGravTimeMult(aMax);
                if (_terminal == TerminalKind.Stop && !_termFlipDone) t += _tFlip;
                return t;
            }

            // Linear interpolation of a per-sample array at parameter u.
            double InterpAt(double u, double[] arr)
            {
                if (u <= _uSamp[0]) return arr[0];
                if (u >= _uSamp[_nSamp - 1]) return arr[_nSamp - 1];
                double segs = Math.Max(1, _spline.SegmentCount);
                double f = u / segs * (_nSamp - 1);
                int i = (int)Math.Floor(f);
                if (i < 0) i = 0; if (i >= _nSamp - 1) i = _nSamp - 2;
                double t = f - i;
                return arr[i] * (1.0 - t) + arr[i + 1] * t;
            }

            // Speed-schedule lookup. NOT InterpAt: the table must be interpolated on v^2, not v.
            //
            // BuildSchedule fills _vSamp with brake/accel branches, both satisfying
            //     v[i]^2 = v[i+1]^2 +/- 2*a*ds
            // so v^2 -- not v -- varies LINEARLY between adjacent samples. Interpolating v linearly
            // reads BELOW the true sqrt curve everywhere inside a braking span, and on the LAST span it
            // is pathological: a Stop pins _vSamp[last] = 0 exactly, so the lookup ramps linearly to
            // zero across a whole sample spacing (~2 km on a 100 km straight). Measured: Fulmar 100 km
            // 143.87 -> 121.67 s, Kestrel 92.97 -> 84.15 s, every max_overshoot still 0.00.
            double InterpSpeedAt(double u, double[] vTab)
            {
                if (u <= _uSamp[0]) return vTab[0];
                if (u >= _uSamp[_nSamp - 1]) return vTab[_nSamp - 1];
                double segs = Math.Max(1, _spline.SegmentCount);
                double f = u / segs * (_nSamp - 1);
                int i = (int)Math.Floor(f);
                if (i < 0) i = 0; if (i >= _nSamp - 1) i = _nSamp - 2;
                double t = f - i;
                double va = vTab[i], vb = vTab[i + 1];
                double sq = va * va * (1.0 - t) + vb * vb * t;
                return sq > 0.0 ? Math.Sqrt(sq) : 0.0;
            }

            // Schedule SPATIAL slope dv/ds at parameter u (per metre of arc).
            double ScheduleSlope(double u)
            {
                double segs = Math.Max(1, _spline.SegmentCount);
                double f = u / segs * (_nSamp - 1);
                int i = (int)Math.Floor(f);
                if (i < 0) i = 0; if (i > _nSamp - 2) i = _nSamp - 2;
                double ds = _sSamp[i + 1] - _sSamp[i];
                if (ds < 1e-9) return 0.0;
                return (_vSamp[i + 1] - _vSamp[i]) / ds;
            }

            // ACCURATE remaining arc from u to the end (refines the partial sample, then adds the tail).
            const int ArcRefineSteps = 8;
            double ArcRemainingAt(double u)
            {
                if (_nSamp < 2) return 0.0;
                double segs = Math.Max(1, _spline.SegmentCount);
                double du = segs / (_nSamp - 1);
                int j = (int)Math.Floor(u / du);
                if (j < 0) { j = 0; u = 0.0; }
                if (j >= _nSamp - 1) return 0.0;
                double uEnd = (j + 1) * du;
                Vector3D t;
                Vector3D prev = _spline.Position(u, out t);
                double s = 0.0;
                for (int m = 1; m <= ArcRefineSteps; m++)
                {
                    Vector3D p = _spline.Position(u + (uEnd - u) * (m / (double)ArcRefineSteps), out t);
                    s += (p - prev).Length();
                    prev = p;
                }
                double tail = _totalLen - _sSamp[j + 1];
                if (tail < 0.0) tail = 0.0;
                return s + tail;
            }

            // Max baked curvature from u to the end of the spline.
            double[] _kAhead;
            double MaxCurvatureAhead(double u)
            {
                int last = _nSamp - 2;               // exclusive of the phantom END knot
                if (last < 1) return 0.0;
                if (_kAhead == null)
                {
                    var a = new double[_nSamp];
                    double m = 0.0;
                    for (int i = last; i >= 1; i--) { if (_kSamp[i] > m) m = _kSamp[i]; a[i] = m; }
                    _kAhead = a;
                }
                double segs = Math.Max(1, _spline.SegmentCount);
                double f = u / segs * (_nSamp - 1);
                int i0 = (int)Math.Floor(f);
                if (i0 < 1) i0 = 1;                  // exclusive of the phantom START knot
                if (i0 > last) i0 = last;
                return _kAhead[i0];
            }

            // THE STRAIGHT-RUN PREDICATE -- decides whether the decisive terminal brake may commit.
            bool StraightRunAt(double u, double remArc, double remStop, bool forward)
            {
                return remArc <= 1.02 * remStop + 5.0
                       && MaxCurvatureAhead(u) < 1e-4
                       && forward
                       && u >= _lastCornerU;
            }

            // FINAL LEG: the closest point is past the last interior control point.
            bool FinalLegAt(double u) { return u > Math.Max(1, _spline.SegmentCount) - 1.0; }

            // Curvature + turn axis guarded against the phantom-ENDPOINT singularity.
            double CurvatureWithAxisGuarded(double u, out Vector3D binormal)
            {
                double segCount = _spline.SegmentCount;
                if (segCount >= 1.0)
                {
                    double g = Math.Min(0.02, 0.25 * segCount);
                    if (u < g) u = g;
                    else if (u > segCount - g) u = segCount - g;
                }
                return _spline.CurvatureWithAxis(u, out binormal);
            }

            public void Update(double dt)
            {
                Elapsed += dt;
                if (IsDone) { _att.Update(); return; }
                if (Elapsed > Timeout)
                {
                    _ship.ThrottleForward = 0.0; _ship.StrafeRight = 0.0; _ship.StrafeUp = 0.0;
                    // The OrientationController is SHARED with Fine; hand it back neutral.
                    _att.ResetTracking();
                    _flipLatched = false;
                    IsDone = true; _att.Update(); return;
                }

                IRigidBody rb = _ship.Body;
                Vector3D pos = rb.Position;
                Vector3D v = rb.LinearVelocity;
                double speed = v.Length();
                // Tack costs cos(angle) off the tangent. Derate at the SOURCE so the schedule, brake
                // ceiling and every cap plan against the authority the ship will actually have --
                // otherwise the terminal brake is planned optimistically and overshoots.
                double aMax = _ship.MaxForwardThrust * rb.InvMass * TackCos;
                if (aMax < 1e-6) aMax = 1e-6;
                // SPORT MODE drag-aware braking.
                double aBrake = aMax + (_ship.SportMode ? Math.Max(0.0, _ship.BrakeDragDecel) : 0.0);
                Vector3D gravity = _ship.Gravity;
                double segCount = Math.Max(1, _spline.SegmentCount);
                if (gravity.LengthSquared() > 1e-4) _everHadGravity = true;

                // ---- track the closest point (monotone forward) ---------------------
                double uClose = _spline.ClosestU(pos, _u);
                if (uClose < _u) uClose = _u;     // never walk backward
                _u = uClose;
                Vector3D tanRaw; Vector3D onPath = _spline.Position(_u, out tanRaw);
                Vector3D tangent = tanRaw.LengthSquared() > 1e-12 ? Vector3D.Normalize(tanRaw) : -_ship.Forward;
                double remaining = Math.Max(0.0, ArcRemainingAt(_u));

                // Sneak thrust envelope; see SneakRampM. envFrac multiplies the throttle WRITE, so
                // the physical burn obeys it; aMax/aBrake carry it so the demands stay consistent.
                double envFrac = 1.0;
                if (ThrustFrac < 1.0)
                {
                    double sDone = InterpAt(_u, _sSamp);
                    double dNear = Math.Min(sDone, remaining);
                    double x = SneakRampM > 1.0 ? dNear / SneakRampM : 1.0;
                    if (x > 1.0) x = 1.0;
                    envFrac = ThrustFrac + (1.0 - ThrustFrac) * x * x;
                    aMax = _ship.MaxForwardThrust * rb.InvMass * TackCos * envFrac;
                    if (aMax < 1e-6) aMax = 1e-6;
                    aBrake = aMax + (_ship.SportMode ? Math.Max(0.0, _ship.BrakeDragDecel) : 0.0);
                }

                Vector3D endCP = _spline.ControlPoints[_spline.ControlPoints.Length - 1];

                // ---- gravity-descent latch (planet landing) -------------------------
                if (!_gravDescentHold && _terminal == TerminalKind.Stop && gravity.LengthSquared() > 1e-6)
                {
                    double aUpDown = Math.Min(_ship.MaxUpThrust, _ship.MaxDownThrust) * rb.InvMass;
                    double gMag = gravity.Length();
                    Vector3D gdir = gravity / gMag;
                    double vDown = Vector3D.Dot(v, gdir);
                    if (remaining < 4.0 * GravityHoverRadius
                        && gMag > HoverRcsHoldFrac * aUpDown && vDown > GravityDescentVelGate)
                        _gravDescentHold = true;
                }

                // ---- terminal hover-hold (gravity landing) --------------------------
                if (_gravDescentHold)
                {
                    Vector3D toCP = endCP - pos;
                    double distCP = toCP.Length();
                    if (distCP < GravityHoverRadius)
                    {
                        if (distCP < GravityHoverArriveD && speed < GravityHoverExitSpd)
                        {
                            _ship.ThrottleForward = 0.0; _ship.StrafeRight = 0.0; _ship.StrafeUp = 0.0;
                            _att.ResetTracking();
                            _flipLatched = false;
                            IsDone = true; _att.Update(); return;
                        }
                        Vector3D aHover = GravityHoverPosGain * toCP - GravityHoverVelGain * v;
                        Vector3D antiG = gravity.LengthSquared() > 1e-12 ? -Vector3D.Normalize(gravity) : -tangent;
                        Vector3D hNose; double hThr;
                        MathHelpers.GravityCompensatedCommand(aHover, gravity, aMax, antiG, out hNose, out hThr);
                        _att.OmegaTrack = null;     // fixed target: bang/PD settles it
                        _att.Target = MathHelpers.LookAlong(hNose);
                        double hErr = MathHelpers.AngleBetween(_ship.Forward, hNose);
                        _ship.ThrottleForward = hThr * Gate(hErr);
                        _ship.StrafeRight = 0.0; _ship.StrafeUp = 0.0;
                        DbgPhase = "hover";
                        _att.Update(); return;
                    }
                }

                // ---- terminal handoff (vacuum Stop) ---------------------------------
                double distEnd = (endCP - pos).Length();

                // RUNAWAY GUARD (speed-agnostic, so it works on a 50k-limit server). If the ship
                // is moving AWAY from the endpoint and getting FURTHER for several seconds, the
                // terminal brake has failed to arrest it -- the flip-to-brake stalled at cruise
                // and it is receding (observed 221 km -> 322 km at 727 m/s). Cutting throttle and
                // asking dampeners to brake stops the bleed; RunawayBraking then reports up so the
                // host faults and the queue rebuilds from rest, rather than sailing to infinity.
                // Positive = moving AWAY from the endpoint (receding).
                double recedeRate = -Vector3D.Dot(v, Vector3D.Normalize(endCP - pos + new Vector3D(1e-9)));
                if (_terminal == TerminalKind.Stop && distEnd > RunawayMinDistM && recedeRate > 5.0)
                {
                    _recedeS += dt;
                    if (_recedeS >= RunawaySeconds)
                    {
                        _ship.ThrottleForward = 0.0; _ship.StrafeRight = 0.0; _ship.StrafeUp = 0.0;
                        _ship.DampenersWanted = true;
                        _att.ResetTracking();
                        _flipLatched = false;
                        RunawayBraking = true;
                        DbgPhase = "runaway-brake";
                        _att.Update();
                        return;
                    }
                }
                else _recedeS = 0.0;

                double handoffWindow = Math.Max(80.0, speed * 1.5);
                bool pathExhausted = _u >= segCount - 1e-3 || remaining < handoffWindow;
                // GRAVITY-DESCENT DEADLOCK release: stopped outside the hover radius.
                bool gravStalled = _gravDescentHold && GravStallRelease
                                   && distEnd >= GravityHoverRadius && speed < GravityHoverExitSpd;
                if (_terminal == TerminalKind.Stop && (!_gravDescentHold || gravStalled)
                    && speed < HandoffSpeed
                    && (distEnd < handoffWindow || pathExhausted))
                {
                    _ship.ThrottleForward = 0.0; _ship.StrafeRight = 0.0; _ship.StrafeUp = 0.0;
                    _att.ResetTracking();
                    _flipLatched = false;
                    IsDone = true; _att.Update(); return;
                }

                // ---- reference point with lookahead ---------------------------------
                // Lead the closest point by LeadTau attitude time-constants of travel.
                double tauAtt = 1.0 / Math.Max(1e-3, AttitudeBandwidth());
                double leadDist = LeadTau * tauAtt * Math.Max(speed, 20.0);
                double uLead = AdvanceU(_u, leadDist);
                Vector3D leadBin; double kLead = CurvatureWithAxisGuarded(uLead, out leadBin);
                Vector3D leadTanRaw; Vector3D leadPos = _spline.Position(uLead, out leadTanRaw);
                Vector3D leadTan = leadTanRaw.LengthSquared() > 1e-12 ? Vector3D.Normalize(leadTanRaw) : tangent;

                double vAlongPre = Vector3D.Dot(v, tangent);

                // ---- scheduled speed target -----------------------------------------
                // NO LOOKAHEAD MIN HERE. This used to be
                //     min(InterpSpeedAt(_u), InterpSpeedAt(AdvanceU(_u, tauAtt * speed)))
                // which treats a FUTURE speed limit as a PRESENT one. The correct anticipation of a cap
                // v_f at distance D ahead is sqrt(v_f^2 + 2*a*D) -- the speed you can still decelerate
                // FROM -- not v_f itself, and RuntimeBrakeCeiling below already computes exactly that
                // over every sample ahead. At the terminal the min was actively harmful: once the
                // horizon overran the spline end the read returned _vSamp[last] = 0, i.e. "be stopped
                // NOW" 400 m early. Crane braked to a dead stop 375 m short and parked until timeout.
                // Removing it took the straight suite 8/10 -> 10/10.
                double vSched = InterpSpeedAt(_u, _vSamp);
                double remStop = Math.Min(remaining, (endCP - pos).Length());
                // The BINDING distance-limited carry speed.
                double vBrakeReq = double.MaxValue;
                bool finalLeg = FinalLegAt(_u);
                bool straightRun = StraightRunAt(_u, remaining, remStop, vAlongPre > 0.0);
                DbgStraightRun = straightRun ? 1 : 0;
                if (_terminal == TerminalKind.Stop)
                {
                    // A STRAIGHT terminal run reserves the tighter straight flip coast.
                    bool tightStop = straightRun && finalLeg && !_everHadGravity;
                    double tFlipStop = tightStop ? _tFlipStraight : _tFlip;
                    double bmStop = tightStop ? _bmStraight : BrakeMargin;
                    double vTrigger = BrakeFeasibleSpeed(bmStop * remStop, aBrake, tFlipStop);
                    double vTermCap = vTrigger;
                    if (straightRun)
                    {
                        // The brake TRIGGERS at the flip-aware feasible speed, then commits.
                        if (!_termFlipDone && vAlongPre >= vTrigger - 0.5)
                        {
                            _termFlipDone = true;
                            _brakeLatchSpeed = vAlongPre;   // never re-accelerate past this
                        }
                        if (_termFlipDone)
                        {
                            // Committed: brake along the clean v^2/2a curve, but release the flip reserve
                            // only as the nose actually swings retrograde.
                            double thetaRem = MathHelpers.AngleBetween(_ship.Forward, -tangent);
                            double flipFrac = _slewSpan > 1e-9
                                              ? SlewTimeLocal(thetaRem, _alphaFlip) / _slewSpan : 0.0;
                            if (flipFrac > 1.0) flipFrac = 1.0; else if (flipFrac < 0.0) flipFrac = 0.0;
                            if (!TermFlipContinuous)
                                flipFrac = Vector3D.Dot(_ship.Forward, tangent) < -0.5 ? 0.0 : 1.0;
                            double vCommitted = BrakeFeasibleSpeed(bmStop * remStop, aBrake,
                                                                   flipFrac * tFlipStop);
                            double vStop = Math.Min(vCommitted, _brakeLatchSpeed);
                            vTermCap = vStop;
                            if (vSched > vStop) vSched = vStop;
                        }
                        else if (vSched > vTrigger) vSched = vTrigger;
                    }
                    else
                    {
                        // Curved/multi-waypoint approach: per-tick flip-aware cap.
                        if (vSched > vTrigger) vSched = vTrigger;
                    }
                    if (vTermCap < vBrakeReq) vBrakeReq = vTermCap;
                }
                // RUNTIME BRAKE CEILING: the binding fix for the spline-knot overshoot.
                double vBrakeCeil = RuntimeBrakeCeiling(_u, aMax, tangent);
                if (vSched > vBrakeCeil) vSched = vBrakeCeil;
                if (vBrakeCeil < vBrakeReq) vBrakeReq = vBrakeCeil;
                DbgVSched = vSched;

                // ---- decompose the velocity error -----------------------------------
                double vAlong = Vector3D.Dot(v, tangent);
                AlongTrackSpeed = vAlong;
                // The tracked reference. The mod adds a dodge sidestep here; with no voxel API the
                // probe is absent and the displacement is identically zero, so this is the on-path point.
                Vector3D refPos = onPath;
                Vector3D crossOffset = pos - refPos;       // perpendicular position error
                crossOffset -= Vector3D.Dot(crossOffset, tangent) * tangent;
                Vector3D vCross = v - vAlong * tangent;     // perpendicular velocity
                DbgCross = crossOffset.Length();

                // ---- assemble the desired acceleration -------------------------------
                // (a) tangential: drive vAlong toward vSched with FEEDFORWARD + proportional.
                double aFF = 0.0;
                if (straightRun)
                {
                    aFF = vAlong * ScheduleSlope(_u);
                    if (_terminal == TerminalKind.Stop && remStop > 1e-3)
                    {
                        bool tightShort = finalLeg && !_everHadGravity;
                        double bmShort = tightShort ? _bmStraight : BrakeMargin;
                        double aStop = -(vAlong * vAlong) / (2.0 * Math.Max(1.0, bmShort * remStop));
                        double tFlipShort = tightShort ? _tFlipStraight : _tFlip;
                        double vBrake = BrakeFeasibleSpeed(bmShort * remStop, aBrake, tFlipShort);
                        // LATCH the stop-brake. vAlong >= vBrake is the right test for STARTING to brake
                        // but not for CONTINUING: the moment braking pulled the ship back under the
                        // feasible speed aStop was dropped, leaving an asymptote rather than an arrival.
                        if (vAlong >= vBrake) _stopBrakeLatched = true;
                        if (_stopBrakeLatched && aStop < aFF) aFF = aStop;
                    }
                }
                double vErr = vSched - vAlong;
                double aTan = aFF + AttitudeBandwidth() * vErr;   // feedforward + 1/tau pull
                if (aTan > aMax) aTan = aMax; else if (aTan < -aMax) aTan = -aMax;
                // Fuel brake-reserve: coast clamps only POSITIVE demand; force-brake commands retrograde.
                if (FuelForceBrake && vAlong > 0.0) aTan = -aMax;
                else if (FuelCoast && aTan > 0.0) aTan = 0.0;
                double aTanIntent = aTan;

                // (b) centripetal: hold the path's curvature.
                Vector3D nearBin; double kNear = CurvatureWithAxisGuarded(_u, out nearBin);
                Vector3D centNear = (kNear > 1e-9 && nearBin.LengthSquared() > 1e-12)
                    ? Vector3D.Cross(nearBin, tangent) : Vector3D.Zero;
                Vector3D centLead = (kLead > 1e-9 && leadBin.LengthSquared() > 1e-12)
                    ? Vector3D.Cross(leadBin, leadTan) : Vector3D.Zero;
                Vector3D centDir = centNear + centLead;
                if (centDir.LengthSquared() > 1e-12) centDir = Vector3D.Normalize(centDir);
                double vCentSpd = speed;
                double aCent = Math.Max(kNear, kLead) * vCentSpd * vCentSpd;
                Vector3D aCentVec = aCent * centDir;

                // (c) cross-track correction: an ACTIVE PD that pulls the ship ONTO the path.
                double crossMag = crossOffset.Length();
                double crossBig = Math.Max(60.0, 0.005 * _totalLen);
                double brakeExcess = (_terminal == TerminalKind.Stop && finalLeg
                                      && crossMag > crossBig) ? (vAlong - vSched) : 0.0;
                // HYSTERESIS, NOT A BARE THRESHOLD. This gated on `vAlong > HandoffSpeed`, and during an
                // off-path terminal brake vAlong parks right AT HandoffSpeed. Measured on Fulmar
                // spline_S_curve: vAlong oscillated 24.98/25.14 across 25.0, flipping posGainScale
                // 1.0 <-> 0.15 every few ticks; against a ~500 m cross error that swung the
                // perpendicular demand 14x and the nose demand 87 deg, so the brake alignment gate held
                // throttle at zero. The ship then could not decelerate and stayed pinned at 25 m/s.
                // A linear BLEND was tried twice and measured WORSE both times: a blend necessarily
                // raises the gain near the boundary, which is exactly where the ship parks.
                if (vAlong > HandoffSpeed * 1.05) _posFadeArmed = true;
                else if (vAlong < HandoffSpeed * 0.95) _posFadeArmed = false;
                double posGainScale = 1.0;
                if (brakeExcess > 0.0 && _posFadeArmed)
                {
                    double frac = brakeExcess / Math.Max(1.0, vAlong);
                    posGainScale = 1.0 - 0.85 * Math.Min(1.0, frac / 0.15);
                    if (posGainScale < 0.15) posGainScale = 0.15;
                }
                // LAUNCH cross-track POSITION ramp: no burn toward nowhere on a standstill reorient.
                double launchPosScale = 1.0;
                if (speed < LaunchAlignSpeed)
                    launchPosScale = MathHelper.Clamp((float)(speed / Math.Max(1.0, LaunchAlignSpeed)), 0f, 1f);
                Vector3D aCorr = -launchPosScale * posGainScale * _perpPosGain * crossOffset - _perpVelGain * vCross;

                // Total perpendicular demand = centripetal (preview FF) + correction.
                Vector3D aPerp = aCentVec + aCorr;
                double aPerpMag = aPerp.Length();
                double aPerpCap = aMax * CentReserve;
                // ---- TERMINAL ARREST -------------------------------------------------
                if (_terminal == TerminalKind.Stop && finalLeg
                    && vAlong > HandoffSpeed && aTanIntent < -0.05 * aMax) _termArrest = true;
                if (_termArrest)
                {
                    // (a) PRIORITY, not a coefficient: the stop demand claims the budget first.
                    double aReq = (vAlong * vAlong) / (2.0 * Math.Max(1.0, remStop));
                    if (aReq > aBrake) aReq = aBrake;
                    double perpSpare = Math.Sqrt(Math.Max(0.0, aBrake * aBrake - aReq * aReq));
                    // (a2) arresting a ship means zeroing its VELOCITY VECTOR, not just vAlong.
                    double perpFloor = ArrestLatFloor
                                       ? aBrake * vCross.Length() / Math.Max(1e-6, speed) : 0.0;
                    if (perpSpare < perpFloor) perpSpare = perpFloor;
                    if (perpSpare < aPerpCap) aPerpCap = perpSpare;
                    // (b) never a tilt whose REVERSAL leaves the thrust gate.
                    double capArrest = aMax * Math.Sin(TerminalNoseCapRad);
                    if (capArrest < aPerpCap) aPerpCap = capArrest;
                }
                if (aPerpMag > aPerpCap && aPerpMag > 1e-9) { aPerp *= aPerpCap / aPerpMag; aPerpMag = aPerpCap; }
                DbgAperp = aPerpMag;

                // Tangential budget left after the perpendicular demand (one drive vector).
                double aTanCap = Math.Sqrt(Math.Max(0.0, aMax * aMax - aPerpMag * aPerpMag));
                if (aTan > aTanCap) aTan = aTanCap; else if (aTan < -aTanCap) aTan = -aTanCap;
                DbgAtanCap = aTanCap;

                // ---- PARTIAL-FLIP corner brake ---------------------------------------
                // Ease the tangential retrograde brake in gradually so a shallow carry-through brake
                // does not demand a full 180 the ship cannot afford mid-corner.
                bool brakeRoom = vBrakeReq >= 1e9 || vAlong < PartialFlipFeasMargin * vBrakeReq;
                double gMagPf = gravity.Length();
                bool lowGravity = !_everHadGravity && gMagPf < PartialFlipMaxGravFrac * aMax;
                bool carryBrake = brakeRoom && lowGravity
                                  && aTanIntent < -0.05 * aMax && aTanIntent > -PartialFlipMaxDepth * aMax
                                  && speed > HandoffSpeed
                                  && !(_terminal == TerminalKind.Stop && straightRun);
                if (carryBrake)
                {
                    if (!_havePfBrake) { _pfBrakeAtan = 0.0; _havePfBrake = true; }
                    double maxStep = aMax * PartialFlipSlewFrac * EffGyroRateCap() * dt;
                    double target = aTan;
                    double ramped = _pfBrakeAtan;
                    if (ramped > target) ramped = Math.Max(target, ramped - maxStep);  // grow negative slowly
                    else ramped = target;                         // backing off the brake is free
                    if (ramped > aTanCap) ramped = aTanCap; else if (ramped < -aTanCap) ramped = -aTanCap;
                    _pfBrakeAtan = ramped;
                    aTan = ramped;
                }
                else
                {
                    // CONTINUITY across the depth-gate boundary.
                    _pfBrakeAtan = Math.Min(0.0, aTan);
                    _havePfBrake = true;
                }

                // ---- braking near a Stop terminal: freeze the retrograde axis --------
                bool hardBrake = _terminal == TerminalKind.Stop && aTan < -0.5 * aMax && remaining < speed * 3.0;

                // THE NOSE ONLY CHASES WHAT THE LATERAL RCS CANNOT DELIVER. aPerp is serviced by
                // ApplyLateral below regardless of where the nose points, so steering the drive
                // axis into a demand the strafe thrusters already cover buys nothing -- and on a
                // slew-limited hull it costs a minute of gated-throttle turning every time the
                // cross-track PD flicks. Measured on the 39.5 kt miner: nose swung 11 -> 99 deg
                // after every burn to chase 0.14 m/s^2 of cross correction that 0.46 m/s^2 of RCS
                // was already delivering; the leg ran at a ~2% burn duty cycle. Only the excess
                // beyond RCS authority tilts the nose.
                double aLatAuth = _ship.MaxLateralThrust * rb.InvMass;
                DbgAtan = aTan;
                Vector3D aDesired = aTan * tangent;
                if (aPerpMag > aLatAuth && aPerpMag > 1e-9)
                    aDesired += aPerp * ((aPerpMag - aLatAuth) / aPerpMag);

                // ---- TACK: hold the drive off the track ------------------------------
                // Applied LAST and as a pure rotation, so the magnitude the caps above negotiated is
                // untouched and every downstream interlock still measures err against the nose it
                // actually gets. The trajectory excursion is a side effect to be bounded, not the
                // point -- the emission lobe rides the hull, so canting the nose is the whole trick.
                // FAIL-SAFE: no cant through the terminal. The brake gate zeroes the drive at 15 deg of
                // nose error, the cone's precession spends part of that budget on tracking lag, and a
                // brake that gates its own throttle off does not recover. Ramped, because a step back
                // to zero is itself a 30 deg jump that would trip the same gate.
                // Sneak keeps the cant through the committed brake: the brake plume is the one
                // that shines at the DESTINATION, and the arrest phase is most of the brake. The
                // hardBrake terminal still straightens -- by then the envelope has the drive at
                // the quiet floor anyway.
                double tackWant = (hardBrake || (_termArrest && ThrustFrac >= 1.0)) ? 0.0 : TackAngleRad;
                double tackStep = TackFadeRate * dt;
                if (_tackCur < tackWant) _tackCur = Math.Min(tackWant, _tackCur + tackStep);
                else if (_tackCur > tackWant) _tackCur = Math.Max(tackWant, _tackCur - tackStep);

                if (_tackCur > 1e-6 && aDesired.LengthSquared() > 1e-12)
                {
                    if (!_haveTackFrame)
                    {
                        // Frame fixed at engage. A track-relative one would precess with the tangent
                        // and the cone would wander.
                        Vector3D seed = Math.Abs(tangent.X) < 0.9 ? Vector3D.UnitX : Vector3D.UnitY;
                        Vector3D u = Vector3D.Cross(tangent, seed);
                        if (u.LengthSquared() > 1e-9)
                        {
                            _tackU = Vector3D.Normalize(u);
                            _tackW = Vector3D.Normalize(Vector3D.Cross(tangent, _tackU));
                            _haveTackFrame = true;
                        }
                    }
                    if (_haveTackFrame)
                    {
                        double bias = MathHelper.Clamp(TackBias, 0.0, 1.0);
                        // The (1-bias) rate scaling is not cosmetic: offsetting the circle by `bias`
                        // speeds the azimuth up by 1/(1-bias) at its far side, and this cancels it
                        // exactly, so PEAK nose rate is constant across the whole bias range. Without
                        // it, a high bias slews the nose fast enough to trip the brake gate.
                        //
                        // FROZEN DURING LARGE SLEWS. The cone only earns its precession while the
                        // drive is lit; sweeping the aim through a gated flip drags the target
                        // around at ~0.05 rad/s and a slow hull can never catch the pin -- measured
                        // live: a 150 deg turn crawling at 1 deg per 40 s while the ship drifted
                        // away at 150 m/s. Advance the azimuth only when the nose is close enough
                        // that the throttle gate could open.
                        if (_att.LastAngleRad < 0.35)
                            _tackPhase += 2.0 * Math.PI * dt * (1.0 - bias) / Math.Max(1.0, TackConeSeconds);
                        if (_tackPhase > 2.0 * Math.PI) _tackPhase -= 2.0 * Math.PI;
                        Vector3D dir = Vector3D.Normalize(aDesired);
                        // bias=0 -> unit circle, mean lateral zero. bias=1 -> phase frozen, lat = 2*U,
                        // a fixed pencil. In between the azimuth dwells on the +U side.
                        Vector3D lat = (Math.Cos(_tackPhase) + bias) * _tackU
                                     + Math.Sin(_tackPhase) * _tackW;
                        lat -= Vector3D.Dot(lat, dir) * dir;
                        if (lat.LengthSquared() > 1e-9)
                            aDesired = aDesired.Length()
                                     * (Math.Cos(_tackCur) * dir
                                        + Math.Sin(_tackCur) * Vector3D.Normalize(lat));
                    }
                }

                // gravity comp + nose direction + throttle.
                Vector3D fallback;
                if (hardBrake)
                {
                    Vector3D retro;
                    if (speed > 30.0) { retro = -v / speed; _brakeAxis = retro; _haveBrakeAxis = true; }
                    else retro = _haveBrakeAxis ? _brakeAxis : (speed > 1e-6 ? -v / speed : -tangent);
                    fallback = retro;
                }
                else
                {
                    // NOSE FLIP HYSTERESIS. `aTan >= 0 ? tangent : -tangent` was a bare sign test on a
                    // quantity that SITS AT ZERO by construction: the speed schedule drives valong onto
                    // vsched, so aTan dithers about zero for the whole cruise, and every crossing threw
                    // the nose command 180 degrees.
                    //
                    // Measured on MUNDOZER at 10 Hz: err ran 2.9 -> 179.2 -> 4.1 -> 7.1 between
                    // consecutive samples with omega only ~0.5 rad/s, so the SETPOINT moved, not the
                    // ship. aTan on those samples was +2.28 / +6.47 / +5.34 against -10 to -25 either
                    // side. Each flip also closed the throttle gate (gate=0.00), so the ship coasted,
                    // fell further below schedule, and the demand flipped straight back. That is the
                    // wobble -- the attitude law was tracking faithfully the whole time.
                    //
                    // A 180 flip costs seconds on any real hull, so it must never be commanded off a
                    // demand that is about to change sign again. The DWELL is what does the work here:
                    // the spurious excursions lasted 1-2 samples, while a genuine turnover holds. Same
                    // fix shape as the cross-track gain chatter, where hysteresis worked and blending
                    // made it worse.
                    if (!_haveNoseSign) { _noseSign = aTan >= 0.0 ? 1 : -1; _haveNoseSign = true; }
                    int want = _noseSign;
                    double flipBand = NoseFlipHysteresisFrac * aMax;
                    if (aTan > flipBand) want = 1;
                    else if (aTan < -flipBand) want = -1;
                    if (want != _noseSign)
                    {
                        _noseFlipHold += dt;
                        if (_noseFlipHold >= NoseFlipMinDwellS)
                        { _noseSign = want; _noseFlipHold = 0.0; }
                    }
                    else _noseFlipHold = 0.0;
                    fallback = _noseSign >= 0 ? tangent : -tangent;
                }

                // A demand the drive cannot meaningfully act on must not steer the nose. The nose
                // command is the DIRECTION of aDesired, so when the schedule is satisfied and only
                // residual cross-track PD noise is left, the "demand" points perpendicular -- and a
                // slew-limited hull spends a minute turning toward it with the throttle gated, then
                // a minute turning back. Measured on the 39.5 kt miner: 1-2 s burn dabs at a ~2%
                // duty cycle, the leg flown at 28 m/s. Below the floor hold the phase's fallback
                // nose (pro/retrograde); ApplyLateral still services the perpendicular via RCS.
                Vector3D nose; double thr;
                double noseFloor = Math.Min(NoseDemandFloorFrac * aMax, NoseDemandFloorAbsMax);
                bool belowFloor = gravity.LengthSquared() < 1e-8 && aDesired.Length() < noseFloor;
                if (belowFloor)
                {
                    nose = fallback;
                    thr = 0.0;
                }
                else
                    MathHelpers.GravityCompensatedCommand(aDesired, gravity, aMax, fallback, out nose, out thr);
                _att.Target = MathHelpers.LookAlong(nose);

                DbgNoseFloor = belowFloor ? 1 : 0;
                {
                    double aMag = aDesired.Length();
                    double tanPart = Math.Abs(aTan);
                    DbgPerpFrac = aMag > 1e-9 ? Math.Max(0.0, 1.0 - tanPart / aMag) : 0.0;
                    if (_dbgHavePrevNose && dt > 1e-6 && nose.LengthSquared() > 1e-12)
                    {
                        double c = MathHelper.Clamp(
                            (float)Vector3D.Dot(Vector3D.Normalize(nose), _dbgPrevNose), -1f, 1f);
                        DbgNoseSweep = Math.Acos(c) * 180.0 / Math.PI / dt;
                    }
                    if (nose.LengthSquared() > 1e-12)
                    { _dbgPrevNose = Vector3D.Normalize(nose); _dbgHavePrevNose = true; }
                }

                // ---- attitude rate-track feedforward ---------------------------------
                // The nose tracks a target sweeping at v*kappa about the binormal.
                double phiDot = kLead * speed;
                Quaternion invO = Quaternion.Conjugate(rb.Orientation);
                // SIGN: +leadBin, not -leadBin. Looks like it should be negated, but this is the
                // convention the rate-track branch consumes; flipping it anti-damps the loop.
                // THE GATE IS A SWEEP RATE, NOT AN EPSILON. kLead on a numerically "straight"
                // Catmull-Rom leg carries curvature noise ~1e-7, so at cruise speed phiDot crossed
                // 1e-6 from noise alone and the proportional rate-track branch preempted the
                // min-time law on straight legs -- the same straight-leg trap as the mod's
                // OmegaTrack=Zero bug, resurfacing through the curvature path. On the 39.5 kt
                // miner that pinned every slew to bandwidth*angle ~ 0.01 rad/s: measured omega
                // plateaus at 0.012 while the plant could do 15x that. Below a sweep the hull
                // would not even notice, the min-time law owns the nose.
                // GATE ON CURVATURE ITSELF, not curvature*speed. phiDot = kLead*speed, so a
                // sweep-rate gate is SPEED-DEPENDENT: numerical curvature noise (~1e-5) that is
                // harmless at 140 m/s crosses 0.002 at ~686 m/s and the rate-track branch preempts
                // the terminal FLIP -- the nose never reaches retrograde, throttle stays gated, and
                // the ship sails through the target at ~1 km/s (measured live, 4 km closest on a
                // 400 km leg). A real corner has kLead >> CurvatureTrackMin; a straight leg's noise
                // never does, at ANY speed. Require real curvature AND a meaningful sweep.
                if (kLead > CurvatureTrackMin && phiDot > OmegaTrackMinSweep
                    && leadBin.LengthSquared() > 1e-12)
                    _att.OmegaTrack = Vector3D.Transform(phiDot * leadBin, invO);
                else
                    // null, NOT Vector3D.Zero: Zero is a value, so OmegaTrack.HasValue stays true and
                    // the rate-track branch keeps preempting the min-time law on a straight leg.
                    _att.OmegaTrack = null;
                DbgKLead = kLead;
                DbgLaw = _att.LastLaw;
                DbgAlphaObs = _att.LastAlphaObserved;
                DbgRateCeil = _att.LastRateCeiling;
                DbgOmegaTrackMag = _att.OmegaTrack.HasValue ? _att.OmegaTrack.Value.Length() : -1.0;
                _att.TrackErrGain = AttitudeBandwidth();
                _att.RollDeadbandScale = kLead > 1e-9 ? 0.0 : 1.0;

                double err = MathHelpers.AngleBetween(_ship.Forward, nose);
                DbgAlignErr = err;
                // BRAKE ALIGNMENT INTERLOCK: hold the drive until the nose is genuinely on retrograde,
                // so the burn stops moving its own aim point. Prograde keeps the wide cos gate.
                double thrGate = (aTan <= 0.0 && err >= BrakeAlignCutoffRad) ? 0.0 : Gate(err);
                // FROM-REST LAUNCH ALIGN: hold the main drive until closely aligned when launching.
                bool launchWeakGrav = gravity.LengthSquared()
                    < (LaunchAlignMaxGravFrac * aMax) * (LaunchAlignMaxGravFrac * aMax);
                if (launchWeakGrav
                    && speed < LaunchAlignSpeed && aTan > 0.0 && err > LaunchAlignRad)
                    thrGate = 0.0;
                // PROGRADE WEAK-GYRO NARROWING: walk the cutoff in as gyro authority falls away.
                if (ProgradeWeakAlignNarrow && aTan > 0.0 && _gyroWeakness > 0.0)
                {
                    double progradeCut = AlignCutoffRad
                        + (ProgradeAlignCutoffWeakRad - AlignCutoffRad) * Math.Min(1.0, _gyroWeakness / 0.3);
                    if (err >= progradeCut) thrGate = 0.0;
                }

                // ---- LARGE-SLEW FLIP LATCH -> the torque-aware bang-bang law ----------
                // The rate-track branch preempts the other laws whenever OmegaTrack has a value, so a
                // big slew on a near-straight leg must clear it to reach the min-time law.
                // null means the rate-track branch is OFF (straight leg) -- zero sweep, the
                // strongest case FOR the latch. MaxValue here disabled the flip latch on exactly
                // the legs that need it, leaving the nose to wobble with aPerp through the slew.
                double sweepSq = _att.OmegaTrack.HasValue
                    ? _att.OmegaTrack.Value.LengthSquared()
                    : 0.0;
                bool flipEligible = FlipBangBangHandoff
                    && sweepSq <= FlipHandoffSweepMax * FlipHandoffSweepMax;
                if (!flipEligible)
                {
                    // Curved leg: rate-track owns the nose, feedforward and all.
                    _flipLatched = false;
                }
                else if (_flipLatched)
                {
                    // Honour a genuine replan, ignore per-tick jitter.
                    if (MathHelpers.AngleBetween(_flipNose, nose) > FlipRelatchRad)
                        _flipNose = nose;
                    // Exit on the LATCHED target: that is the angle this law is actually closing.
                    if (MathHelpers.AngleBetween(_ship.Forward, _flipNose) < FlipExitErrRad)
                        _flipLatched = false;
                }
                else if (err > FlipEnterErrRad)
                {
                    _flipNose = nose;
                    _flipLatched = true;
                }

                if (_flipLatched)
                {
                    _att.Target = MathHelpers.LookAlong(_flipNose);
                    _att.OmegaTrack = null;
                }
                // envFrac converts "fraction of the derated budget" back to physical throttle:
                // thr is normalised by the enveloped aMax, and the drive itself is not derated.
                DbgThr = thr; DbgThrGate = thrGate; DbgEnvFrac = envFrac;
                DbgATan = aTan; DbgAMax = aMax;
                _ship.ThrottleForward = thr * thrGate * envFrac;

                // ---- lateral RCS: deliver the perpendicular demand the nose can't cover
                ApplyLateral(aPerp);

                DbgPhase = aTan < -0.5 * aMax ? "brake" : (aTan > 0.5 * aMax ? "accel" : "track");
                _att.Update();
                DbgGyroCmd = Math.Sqrt(_ship.Pitch * _ship.Pitch
                                     + _ship.Yaw * _ship.Yaw + _ship.Roll * _ship.Roll);
            }

            // Max speed at u0 that can still brake to every upcoming curvature/stop cap.
            double RuntimeBrakeCeiling(double u0, double aMax, Vector3D curTangent)
            {
                double segs = Math.Max(1, _spline.SegmentCount);
                double f0 = u0 / segs * (_nSamp - 1);
                int i0 = (int)Math.Floor(f0);
                if (i0 < 0) i0 = 0; if (i0 > _nSamp - 1) i0 = _nSamp - 1;
                double s0 = InterpAt(u0, _sSamp);
                double alpha = AttitudeBandwidth() * AttitudeBandwidth();
                // Loop-invariant: SlewTimeLocal re-derived this from IShip on every one of the _nSamp
                // iterations. Hoisting is arithmetically inert and is the largest constant factor in
                // the only O(_nSamp) walk left on the per-tick path.
                double omegaEff = EffGyroRateCap();
                double aMaxSq = aMax * aMax;
                double aTanFloor = 0.05 * aMax;
                double ceil = double.MaxValue;
                for (int j = i0 + 1; j < _nSamp; j++)
                {
                    double vcap = _vSamp[j];
                    double aCent = _kSamp[j] * vcap * vcap;
                    double aTanAvail = Math.Sqrt(Math.Max(0.0, aMaxSq - aCent * aCent));
                    if (aTanAvail < aTanFloor) aTanAvail = aTanFloor;
                    double cphi = Vector3D.Dot(curTangent, _tSamp[j]);
                    if (cphi > 1.0) cphi = 1.0; else if (cphi < -1.0) cphi = -1.0;
                    double turn = Math.Acos(cphi);
                    // REORIENTATION coast before braking into this sample.
                    double reorient;
                    if (turn > 0.20 && _kSamp[j] > 1e-6)
                    {
                        // GENUINE CORNER: to brake INTO it the nose must rotate past the turn.
                        double vLocalCap = _vSamp.Length > 0 ? _vSamp[i0] : vcap;
                        double brakeDepth = vLocalCap > 1e-3
                            ? Math.Min(1.0, Math.Max(0.0, (vLocalCap - vcap) / vLocalCap)) : 0.0;
                        reorient = turn + (Math.PI - turn) * brakeDepth;
                    }
                    else reorient = turn;
                    double tTurn = FlipReserveFactor * SlewTimeWith(reorient, alpha, omegaEff);
                    double dsTurn = vcap * tTurn;                 // coast distance during the reorient
                    double dsEff = (_sSamp[j] - s0) - dsTurn;
                    if (dsEff < 0.0) dsEff = 0.0;
                    double vmax = Math.Sqrt(vcap * vcap + 2.0 * aTanAvail * dsEff);
                    if (vmax < ceil) ceil = vmax;
                }
                return ceil;
            }

            // Bang-bang slew time for a theta-rad nose rotation.
            double SlewTimeLocal(double theta, double alpha)
            {
                return SlewTimeWith(theta, alpha, EffGyroRateCap());
            }

            // Same law with the rate cap hoisted, for loops where it is invariant.
            static double SlewTimeWith(double theta, double alpha, double omega)
            {
                if (theta <= 1e-4 || alpha < 1e-9) return 0.0;
                if (omega < 1e-6) omega = 1e-6;
                double thetaSat = (omega * omega) / alpha;
                return theta <= thetaSat ? 2.0 * Math.Sqrt(theta / alpha) : omega / alpha + theta / omega;
            }

            // The SUSTAINABLE nose-sweep / flip rate cap (rad/s) this hull can actually hold.
            double EffGyroRateCap()
            {
                return EffGyroRateCapWith(AttitudeBandwidth());
            }

            // Same law with the attitude bandwidth supplied, so the bake can use its frozen value.
            double EffGyroRateCapWith(double wAtt)
            {
                double omegaCmd = OrientationController.CommandedRateCapBase;
                if (omegaCmd > OrientationController.GyroHardwareRateCap) omegaCmd = OrientationController.GyroHardwareRateCap;
                double hwCap = _ship.GyroRateCap;
                if (hwCap <= 1e-6) hwCap = OrientationController.GyroHardwareRateCap;
                double eff = Math.Min(omegaCmd, Math.Min(hwCap, wAtt));
                return eff > 1e-3 ? eff : 1e-3;
            }

            // Advance u by a target arc-length along the spline (clamped to the end).
            double AdvanceU(double u0, double dist)
            {
                if (dist <= 0.0) return u0;
                double s0 = InterpAt(u0, _sSamp);
                double sTarget = s0 + dist;
                if (sTarget >= _totalLen) return _uSamp[_nSamp - 1];
                int lo = 0, hi = _nSamp - 1;
                while (lo + 1 < hi)
                {
                    int mid = (lo + hi) / 2;
                    if (_sSamp[mid] < sTarget) lo = mid; else hi = mid;
                }
                double ds = _sSamp[hi] - _sSamp[lo];
                double t = ds > 1e-9 ? (sTarget - _sSamp[lo]) / ds : 0.0;
                return _uSamp[lo] * (1.0 - t) + _uSamp[hi] * t;
            }

            // Soft thrust gate: scale by cos(err) inside the cutoff cone, hard-cut beyond.
            static double Gate(double err)
            {
                return err < AlignCutoffRad ? Math.Max(0.0, Math.Cos(err)) : 0.0;
            }

            // Lateral RCS delivers the perpendicular demand the main nose tilt does not cover.
            void ApplyLateral(Vector3D aPerpDemand)
            {
                IRigidBody rb = _ship.Body;
                double aRight = _ship.MaxRightThrust * rb.InvMass;
                double aLeft = _ship.MaxLeftThrust * rb.InvMass;
                double aUp = _ship.MaxUpThrust * rb.InvMass;
                double aDown = _ship.MaxDownThrust * rb.InvMass;
                Vector3D aLatBody = Vector3D.Transform(aPerpDemand, Quaternion.Conjugate(rb.Orientation));
                _ship.StrafeRight = MathHelpers.StrafeCmd(aLatBody.X, aRight, aLeft);
                _ship.StrafeUp = MathHelpers.StrafeCmd(aLatBody.Y, aUp, aDown);
            }
        }
    }
}

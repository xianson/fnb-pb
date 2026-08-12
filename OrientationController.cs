using System;
using VRageMath;

namespace IngameScript
{
    partial class Program
    {
        // Rotates the ship toward Target. Verbatim port of the mod's WaypointQueue/OrientationController.cs;
        // the discrete min-time switching law below has no free parameters and must not be re-tuned.
        public sealed class OrientationController
        {
            public readonly IShip Ship;
            public Quaternion Target = Quaternion.Identity;

            public double SwitchAngleDeg = 10.0;
            public double Kp = 1.0;
            public double Kd = 2.0;

            // Fallback commanded gyro-rate cap, used when the world's own limit cannot be read.
            // A PB can never read it, so this is the normal path here.
            // How far a measured alpha may raise the commanded alpha above the derived one.
            //
            // 1.0 = not at all. The max-hold estimator exists to rescue a PESSIMISTIC derived model,
            // which mattered while the inertia tensor was wrong; with the tensor fixed the derived
            // value is measured accurate to 1.4% (0.881 modelled vs 0.8933 by gyrotest, exactly
            // linear in ControlTorque), so there is nothing left for it to rescue.
            //
            // Any inflation is not free: aDes above the achievable alpha saturates the command every
            // tick, which removes proportional control and leaves bang-bang. Measured at 1.25 the
            // ship stopped spinning but hunted +/-6 deg about the setpoint with aDes pinned at 1.1029
            // against an available 0.88 on every sample.
            public static double AlphaCmdMaxInflation = 1.0;
            public const double CommandedRateCapBase = 1.0;   // rad/s
            public const double GyroHardwareRateCap = Math.PI;   // rad/s

            // Ship-derived planned 180deg flip duration (s).
            public static double DeriveFlipLeadTime(IShip ship)
            {
                Vector3D I = ship.Body.InertiaBody;
                double iMax = Math.Max(I.X, Math.Max(I.Y, I.Z));
                double tau = Math.Max(ship.MaxTorque, 1.0);
                double tBang = 2.0 * Math.Sqrt(Math.PI * iMax / tau);
                double wMax = ship.GyroRateCap;
                if (!(wMax > 1e-6)) wMax = CommandedRateCapBase;
                // Select the profile by whether the rate cap is actually reached, not by Math.Max.
                double alpha = tau / iMax;                       // rad/s^2 about the worst axis
                double wPeak = Math.Sqrt(alpha * Math.PI);       // peak rate of a triangular 180
                double tFlip = wPeak <= wMax
                    ? tBang                                      // cap never binds -> triangular
                    : Math.PI / wMax + wMax / alpha;             // rate-limited -> trapezoidal
                return 1.2 * tFlip;
            }

            public double DeadbandPitch = 0.005;
            public double DeadbandYaw = 0.005;
            public double DeadbandRoll = 0.05;
            double DeadbandPitchEff = 0.005, DeadbandYawEff = 0.005;

            // Scales the roll deadband only (0..1).
            public double RollDeadbandScale = 1.0;

            // The override gyro is a VELOCITY SERVO, not a torque actuator.
            public static bool BrakeableRateCeiling = true;
            // Share of the plant the off-axis damper may claim before the closing axis is served.
            // 1.0 restores the old unbounded-deadbeat behaviour (which starves the aim); 0 disables
            // off-axis damping entirely and lets the ship tumble. See the min-time branch.
            public static double PerpAuthorityFrac = 0.35;
            // Fraction of the achievable alpha the SWITCHING function is allowed to assume. Below 1
            // the brake fires early, trading a little slew time for margin against under-delivery.
            public static double SwitchAlphaMargin = 1.0;
            // Below this nose error the min-time law hands off to a critically damped endgame, so the
            // setpoint is approached smoothly instead of bang-bang. 0 disables the hand-off.
            // 20 measured better than 10 and far better than 0 on MUNDOZER (settled omega RMS
            // 0.051 / 0.087 / 0.280, gyro saturated 0% / 1% / 64% of samples).
            public static double HandoffAngleDeg = 20.0;
            // Shortest time the law is allowed to plan for reaching the rate cap. Bounds commanded
            // angular acceleration on hulls whose authority far exceeds what a 60 Hz loop can meter.
            // 0 disables the bound.
            public static double MinSlewSeconds = 0.25;
            public static double AlphaObservedDecay = 0.995;
            public static double AlphaObservedMinFrac = 0.25;
            // Angular acceleration the law asked for last tick. Only used to decide whether a low
            // measurement is evidence of a weak plant or just evidence that we asked for nothing.
            double _lastDemand;
            // |omega_perp| last tick, for the growth floor on the perpendicular damper. Stale across
            // a gap in the min-time branch, which the Min(.., alphaPerpAvail) bound contains.
            double _prevPerpMag;

            // SE'S FREE ANGULAR SLOWDOWN, as a first-order rate. MyGridGyroSystem applies
            // SlowdownTorque along -omega on every axis where Sign(omega) != Sign(ControlTorque), so
            // the plant is a double integrator while ACCELERATING and wdot = -a - k*w while BRAKING.
            //
            // Measured by free decay on MUNDOZER, 2026-08-12: exponential to four digits over a 100:1
            // range of w, k = 1.9864 by 0.2 s stride ratios and 1.9845 by inverting the tool's own
            // 0.1 s forward difference. That is SLOWDOWN_FACTOR_TORQUE_MULTIPLIER_LARGE_SHIP = 2.0
            // without the 0.93 MAX_SLOWDOWN factor.
            //
            // DEFAULTED TO ZERO -- the double integrator -- BECAUSE 1.986 OVERSHOT THE FLIP IN FLIGHT.
            //
            // The measurement is sound but it was taken in FREE DECAY, at ControlTorque = 0. Using it
            // in the switching function assumes the slowdown is ADDITIVE to a commanded brake
            // (wdot = -a - k*w). That is unvalidated, and there is a concrete reason to doubt it: the
            // gyro system allocates slowdown out of the SAME finite torque budget as the command, so
            // under a full-authority brake the two may not sum. On SUNDIAL at the flip (a = 0.729,
            // w0 = 1.68) additive drag says brake at 30 deg to go where the double integrator says
            // 111 deg -- a 3.7x difference, and the ship sailed through.
            //
            // Do not re-enable this from the free-decay number. The measurement that settles it is
            // decel WHILE COMMANDING A BRAKE: additive predicts a + k*w, shared predicts ~max(a, k*w).
            public static double SlowdownRatePerSec = 0.0;

            // Angle swept while braking from |v| to rest at max decel a against linear drag k.
            //
            //   w(t) = (w0 + a/k)e^(-kt) - a/k,  t_stop = (1/k)ln(1 + k*w0/a)
            //   Theta = w0/k - (a/k^2) ln(1 + k*w0/a)
            //
            // Closed form, no approximation. The series is w0^2/(2a) - k*w0^3/(3a^2) + ..., so it
            // reduces to the double integrator as k->0 and drag always SHORTENS the stop. Using
            // v^2/(2a) therefore over-estimates the stopping angle -- 5% at 0.5 rad/s rising to 26%
            // at the rate cap on this hull -- and fires the brake that much early, which is what
            // hands the endgame a residual to clean up.
            static double StopAngle(double v, double a, double k)
            {
                if (!(a > 1e-9)) return 0.0;
                double w = Math.Abs(v);
                if (!(k > 1e-9)) return w * w / (2.0 * a);
                double x = k * w / a;
                // Below this the closed form is w/k minus something almost equal to it; the series is
                // the accurate branch, not the cheap one.
                if (x < 0.05) return w * w / (2.0 * a) * (1.0 - 2.0 * x / 3.0);
                return w / k - (a / (k * k)) * Math.Log(1.0 + x);
            }
            double _prevOmegaMag;
            double _alphaObserved;

            // Zero the command once the nose has arrived and is nearly still.
            public static bool TerminalLatch = true;
            public static double TerminalLatchAngleRad = 1.0 * Math.PI / 180.0;
            public static double TerminalLatchRate = 0.05;   // rad/s

            // DEBUG HOOK (harness only, strip before release): (angleRad, aU, |aDes|, |dw|, |omega|).
            // Debug hook, null unless a host asks for it. Widened to carry the SWITCHING state (v, s,
            // sBand, alphaSwitch) because aU alone cannot distinguish "decided not to brake yet" from
            // "never evaluated the brake": both look like aU = +alphaCmd. Array rather than a 10-arg
            // delegate; it only allocates on ticks where tracing is already on.
            // Order: angleRad, aU, aDesLen, rateGap, omegaLen, v, s, sBand, alphaSwitch, alphaCmd.
            public static Action<double[]> TraceMinTime;

            public Vector3D? OmegaTrack = null;     // planned body-frame rate (rad/s); null = off

            // World direction the ship's UP should lean toward. The forward-error laws below are
            // BLIND to roll about the forward axis -- cross(target, forward) has no forward
            // component -- so a roll reference used to be measured and reported but never acted
            // on: RollErr sat at 26.8 deg for seven minutes with the aim at exactly 0. This is
            // the missing closure.
            public Vector3D? RollRefWorld = null;
            public double RollGain = 0.6;           // rad/s of command per rad of roll error
            public double RollRateCap = 0.25;       // rad/s
            public double RollDeadbandRad = 0.02;
            public double TrackErrGain = 1.0;
            public double CommandRateCap = 1.0;

            public OrientationController(IShip ship)
            {
                Ship = ship;
            }

            public void ResetTracking()
            {
                _haveTargetHistory = false;
                _setpointRateWorld = Vector3D.Zero;
                OmegaTrack = null;
                TrackErrGain = 1.0;
                RollDeadbandScale = 1.0;
                RollRefWorld = null;
            }

            // Pure dead-zone: zero below band, pass through unchanged above.
            static double ApplyDeadband(double cmd, double band)
            {
                return Math.Abs(cmd) < band ? 0.0 : cmd;
            }

            // PLANT: a = (omega_cmd - omega) * 60/rampFrames, torque = a*I clamped to a sphere, with
            // rampFrames = (|dw|^2 > 2.467401 ? 120 : floor(|dw|^2*48.228886)+1) -- so gain is
            // NON-monotonic in the gap: a big command inflates rampFrames and throttles itself.
            public static bool InversePlant = true;
            // WHICH ACTUATION PATH THIS CONTROLLER DRIVES. The rampFrames schedule above is a property
            // of SE's GYRO-OVERRIDE VELOCITY SERVO, which is what a PB drives. Indicator injection
            // (the plugin) writes ControlTorque directly and there is NO ramp -- the adapter reopens
            // the gap at a flat 60 Hz, so the law must close it at a flat 60 Hz too.
            //
            // Inverting the wrong plant is not a small gain error. The fixed point is not a
            // contraction, so above |aDesired| = 60/sqrt(48.228886) = 8.6399 rad/s^2 the three
            // iterations diverge and the returned gap is 26-120x too large:
            //
            //     aDes  8.60 -> dw 0.143 -> adapter reconstructs   8.60   (exact)
            //     aDes  8.64 -> dw 3.744 -> adapter reconstructs 224.6    (26x)
            //     aDes 11.06 -> dw 14.93 -> adapter reconstructs 896      (81x)   <- MUNDOZER yaw
            //
            // Saturating the MAGNITUDE would be survivable; the bang-bang branch commands full
            // authority anyway. What it actually costs is the AXIS: |control| reaches ~4.3 and the
            // Pitch/Yaw/Roll setters clamp each component INDEPENDENTLY, so the commanded rotation
            // axis is rewritten every tick as omegaBody moves. That is the "very high oscillation on
            // the flip axes" seen on the server -- and it is why the Pickaxe (alpha 0.88, far below
            // the cliff) flew this same code perfectly.
            public static bool ServoRampFrames = true;
            // Engine tick (s). Fixed by SE physics; AccelToRateGap inverts a 60 Hz servo ramp.
            public const double TickSeconds = 1.0 / 60.0;
            // PB DEVIATION: in the mod the law ran every engine tick, so the zero-order hold and the
            // engine tick were the same number. A PB holds its command for a whole PB period, so the
            // ZOH term must use that instead. Set from Runtime.TimeSinceLastRun; equals TickSeconds
            // at Update1, where this reduces to the original code exactly.
            public double ControlPeriod = TickSeconds;
            public static double SetpointRateSmooth = 0.1;
            Vector3D _prevTargetDirWorld;
            bool _haveTargetHistory;
            Vector3D _setpointRateWorld;

            // Solves omega_cmd = omega + a*rampFrames(|dw|)/60 for dw.
            Vector3D AccelToRateGap(Vector3D aDesired)
            {
                // No servo, no ramp: the gap that yields aDesired over the period the command is
                // ACTUALLY held for. Nominally 1/60, but a sim hitch holds it for several frames and
                // the gap has to grow with them -- otherwise the adapter reopens a one-tick gap that
                // the plant then integrates for n ticks, delivering n times the demand. The servo
                // branch below keeps the fixed 60: rampFrames is the engine's own schedule and does
                // not stretch with a hitch.
                double period = ControlPeriod > 1e-6 ? ControlPeriod : TickSeconds;
                if (!ServoRampFrames) return aDesired * period;
                Vector3D dw = aDesired / 60.0;          // rampFrames = 1 seed
                for (int i = 0; i < 3; i++)
                {
                    double s = dw.LengthSquared();
                    int rf = s > 2.467401 ? 120 : (int)(s * 48.228886) + 1;
                    Vector3D next = aDesired * (rf / 60.0);
                    if ((next - dw).LengthSquared() < 1e-18) { dw = next; break; }
                    dw = next;
                }
                return dw;
            }

            public void Update()
            {
                IRigidBody rb = Ship.Body;
                double h = ControlPeriod > 1e-6 ? ControlPeriod : TickSeconds;

                Vector3D targetDir = Vector3D.Transform(-Vector3D.UnitZ, Target);
                if (targetDir.LengthSquared() < 1e-12)
                {
                    Ship.Pitch = 0.0; Ship.Yaw = 0.0; Ship.Roll = 0.0;
                    return;
                }
                targetDir = Vector3D.Normalize(targetDir);

                // Measure the setpoint's own rotation rate.
                if (_haveTargetHistory)
                {
                    Vector3D dTheta = Vector3D.Cross(_prevTargetDirWorld, targetDir);   // rad this period
                    Vector3D rateNow = dTheta / h;                                      // rad/s
                    // Both filter constants are per-engine-tick in the original. Re-derive them for the
                    // actual period so a slower PB cadence keeps the same time constant; at h = 1/60
                    // these are exactly SetpointRateSmooth and AlphaObservedDecay.
                    double ticks = h * 60.0;
                    double smooth = 1.0 - Math.Pow(1.0 - SetpointRateSmooth, ticks);
                    _setpointRateWorld += (rateNow - _setpointRateWorld) * smooth;
                }
                _prevTargetDirWorld = targetDir;
                _haveTargetHistory = true;

                Vector3D currentFwd = Ship.Forward;
                if (currentFwd.LengthSquared() < 1e-12)
                {
                    Ship.Pitch = 0.0; Ship.Yaw = 0.0; Ship.Roll = 0.0;
                    return;
                }
                currentFwd = Vector3D.Normalize(currentFwd);

                // World-frame torque axis; magnitude = sin(angle).
                Vector3D errorWorld = Vector3D.Cross(targetDir, currentFwd);

                // 180-deg guard: cross is zero, so the ship can't flip forward->retro.
                if (errorWorld.LengthSquared() < 0.001 && Vector3D.Dot(targetDir, currentFwd) < 0)
                {
                    errorWorld = Vector3D.Transform(Vector3D.UnitY, rb.Orientation);
                }

                Quaternion invOrient = Quaternion.Conjugate(rb.Orientation);
                Vector3D errorBody = Vector3D.Transform(errorWorld, invOrient);
                Vector3D omegaBody = Vector3D.Transform(rb.AngularVelocity, invOrient);

                // Measured angular accel, maintained every tick regardless of which law runs below.
                {
                    double omMagNow = omegaBody.Length();
                    double measuredNow = Math.Abs(omMagNow - _prevOmegaMag) / h;
                    _prevOmegaMag = omMagNow;
                    // DECAY ONLY UNDER DEMAND. A max-hold that decays whenever the measurement is
                    // low manufactures a capability reduction out of the ABSENCE of evidence: sitting
                    // near the setpoint, the law commands 1-2% of authority, so of course |dw|/h is
                    // small -- that says nothing about what the hull could do if asked.
                    //
                    // It is not harmless, because alphaMax (below) is min(derived, observed): a
                    // decaying observation drags the SWITCH down while the COMMAND stays sized off
                    // the derived plant, so the brake fires earlier and earlier the longer the ship
                    // holds station. Measured on MUNDOZER in the endgame: aSw bled 0.674 -> 0.349 over
                    // ~2.5 s, a clean exponential, then snapped back to 0.728 on the next real slew,
                    // while aCmd sat at 0.729 throughout.
                    //
                    // A shortfall is only evidence when we asked for more than we got. Ask for less
                    // than the current estimate and the low reading is the expected outcome, not a
                    // measurement of the plant.
                    if (measuredNow > _alphaObserved) _alphaObserved = measuredNow;
                    else if (_lastDemand > _alphaObserved)
                        _alphaObserved *= Math.Pow(AlphaObservedDecay, h * 60.0);
                }

                double dot = Vector3D.Dot(targetDir, currentFwd);
                if (dot > 1.0) dot = 1.0;
                if (dot < -1.0) dot = -1.0;
                double angleRad = Math.Acos(dot);
                double angleDeg = angleRad * 180.0 / Math.PI;
                LastAngleRad = angleRad;
                // Diagnostics only, read by the adapter's 60 Hz dump. Statics because the adapter has
                // no handle on the controller, and the plugin flies one ship at a time.
                DbgAngleRad = angleRad;
                DbgRollCmd = 0.0;          // set below only if the roll closure actually ran
                DbgPerpRaw = 0.0;
                DbgPerpCapped = 0.0;

                Vector3D control;
                if (OmegaTrack.HasValue)
                {
                    // Terminal latch: arrived and nearly still -> command exactly zero rate.
                    if (TerminalLatch
                        && OmegaTrack.Value.LengthSquared() < 1e-12
                        && angleRad < TerminalLatchAngleRad
                        && omegaBody.LengthSquared() < TerminalLatchRate * TerminalLatchRate)
                    {
                        Ship.Pitch = 0.0; Ship.Yaw = 0.0; Ship.Roll = 0.0;
                        return;
                    }
                    // Rate track (velocity-servo plant).
                    LastLaw = "rate";
                    double errLenT = errorBody.Length();
                    Vector3D axis = errLenT > 1e-9 ? errorBody / errLenT : Vector3D.Zero;
                    double omegaErr = TrackErrGain * angleRad;
                    if (omegaErr > CommandRateCap) omegaErr = CommandRateCap;
                    if (BrakeableRateCeiling)
                    {
                        Vector3D Ib = rb.InertiaBody;
                        double iEffR = axis.X * axis.X * Ib.X
                                     + axis.Y * axis.Y * Ib.Y
                                     + axis.Z * axis.Z * Ib.Z;
                        // Same degenerate-direction trap as the min-time branch below: 1.0 is not an
                        // inertia, and MaxTorque/1.0 is not a plant. Use the smallest real principal
                        // inertia so the derived alpha stays physical.
                        if (iEffR < 1.0) iEffR = Math.Max(1.0, Math.Min(Ib.X, Math.Min(Ib.Y, Ib.Z)));
                        double alphaDerived = Ship.MaxTorque / iEffR;

                        double alphaEff = _alphaObserved >= alphaDerived * AlphaObservedMinFrac
                            ? _alphaObserved
                            : alphaDerived;
                        if (alphaEff > 1e-6)
                        {
                            double omBrakeable = Math.Sqrt(2.0 * alphaEff * angleRad);
                            if (omegaErr > omBrakeable) omegaErr = omBrakeable;
                            LastRateCeiling = omBrakeable;
                        }
                    }
                    Vector3D omegaCmdT = OmegaTrack.Value + omegaErr * axis;
                    double inv = CommandRateCap > 1e-6 ? 1.0 / CommandRateCap : 0.0;
                    control = omegaCmdT * inv;
                }
                // The min-time law is valid all the way to zero error, so it does not hand off to PD.
                else if (InversePlant || angleDeg > SwitchAngleDeg)
                {
                    double errorLen = errorBody.Length();
                    Vector3D errorDir = errorLen > 1e-3
                        ? errorBody / errorLen
                        : Vector3D.Zero;

                    double angVelMag = omegaBody.Length();
                    Vector3D I = rb.InertiaBody;
                    double iEff = errorDir.X * errorDir.X * I.X
                                + errorDir.Y * errorDir.Y * I.Y
                                + errorDir.Z * errorDir.Z * I.Z;
                    // DEGENERATE DIRECTION -> A REAL INERTIA, NOT 1.0.
                    //
                    // errorDir is set to Vector3D.Zero once errorLen <= 1e-3, so at the setpoint iEff
                    // is 0 and this guard fired. `1.0` is a divide-by-zero guard, but it is also a
                    // kg*m^2, and MaxTorque/1.0 is then a plant that does not exist: logged live as
                    // alphaDerived 3.6e8 and alphaCmd 3.6e8 rad/s^2. Nothing downstream clamps against
                    // a ceiling that large, so the perpendicular deadbeat term (aDes = -omegaPerp/h)
                    // went out at 1.2-2.1 rad/s^2 against an available 0.88 -- permanent saturation
                    // exactly where the ship is trying to hold still, i.e. the setpoint jitter.
                    //
                    // The smallest principal inertia is the most optimistic axis the hull actually
                    // has, so it bounds alpha by something physical while staying conservative about
                    // NOT under-driving. Any real axis would do; none of them is 1.
                    if (iEff < 1.0) iEff = Math.Max(1.0, Math.Min(I.X, Math.Min(I.Y, I.Z)));
                    // Prefer the MEASURED capability for the brake point.
                    double alphaDerivedBB = Ship.MaxTorque / iEff;
                    double alphaFloorBB = alphaDerivedBB * AlphaObservedMinFrac;
                    double alphaMeasBB = _alphaObserved > alphaFloorBB ? _alphaObserved : alphaFloorBB;
                    double alphaMax = Math.Min(alphaDerivedBB, alphaMeasBB);
                    if (alphaMax < 1e-3) alphaMax = 1e-3;
                    double brakingAngle = angVelMag * angVelMag / (2.0 * alphaMax);
                    LastLaw = InversePlant ? "mintime" : "bangbang";
                    LastRateCeiling = Math.Sqrt(2.0 * alphaMax * angleRad);

                    if (InversePlant)
                    {
                        // Discrete-time minimum-time slew. Along the closing axis u this is a double
                        // integrator; minimum-time control is u_c = -alpha*sign(s) on the switching
                        // function s = theta + v|v|/(2 alpha). The v*h/2 term is the zero-order-hold
                        // correction and is the whole point: sampling the continuous switching curve
                        // directly fires the brake a tick late, overshoots and limit-cycles.
                        // Free parameters: none.
                        Vector3D u = -errorDir;
                        // Brake on the CONSERVATIVE alpha (early is safe), command the GENEROUS one.
                        // COMMAND NO MORE THAN THE PLANT CAN DELIVER.
                        //
                        // _alphaObserved is a MAX-HOLD of |d|omega||/h with only a slow decay, so on a
                        // network-quantised omega a single-tick jump latches it high and it stays
                        // there. Measured in flight: alphaDerived 0.88 rad/s^2 (validated against
                        // gyrotest at 0.8933, 1.4%) while _alphaObserved had latched at 6.6 -- 7.5x.
                        //
                        // Letting that through is not merely optimistic, it destroys the loop. aDes
                        // feeds AccelToRateGap as aDes/60, so a 6.6 demand asks the rate loop for a
                        // 0.11 rad/s gap and 6.6 rad/s^2 of acceleration from a plant that can give
                        // 0.88. The command saturates every tick in both directions, proportional
                        // control disappears entirely, and what is left is bang-bang -- which is
                        // precisely what overshoots and chatters at the setpoint. Observed live as
                        // aU alternating +6.4, -5.7, -3.5, +5.0 at ang = 0.1 deg.
                        //
                        // The max() exists so a PESSIMISTIC derived model can still be corrected
                        // upward by observation, which mattered when the inertia tensor was wrong.
                        // Keep that, but bound it: an observation may raise the command a little, not
                        // multiply it. Anything above the plant's real authority is unusable anyway,
                        // because the adapter clamps it to +/-1 on the way out.
                        double alphaCmd = alphaDerivedBB;
                        if (_alphaObserved > alphaCmd) alphaCmd = _alphaObserved;
                        double alphaCmdCeiling = alphaDerivedBB * AlphaCmdMaxInflation;
                        if (alphaCmd > alphaCmdCeiling) alphaCmd = alphaCmdCeiling;

                        // AUTHORITY THE LOOP CAN ACTUALLY METER. Some hulls are absurdly strong on
                        // one axis: measured 77 rad/s^2 in roll on a 19-gyro miner (1.494e9 N.m over
                        // an inertia of 1.92e7) against 2.5 in pitch and yaw. At 60 Hz that is
                        // 1.28 rad/s of rate change in a SINGLE TICK, against a rate cap of ~3.46 --
                        // so the law slams to the cap and back within a few ticks for a fraction of
                        // a degree of error, which is felt as jitter rather than control.
                        //
                        // Commanding more than this buys nothing anyway: the ship cannot exceed the
                        // rate cap, so acceleration beyond "reach the cap in MinSlewSeconds" is spent
                        // entirely on overshooting it. Bounded here rather than in the adapter so the
                        // switching curve is computed against the SAME number that gets commanded.
                        if (MinSlewSeconds > 1e-3)
                        {
                            double meterable = Ship.GyroRateCap / MinSlewSeconds;
                            if (meterable > 1e-6 && alphaCmd > meterable) alphaCmd = meterable;
                            if (meterable > 1e-6 && alphaMax > meterable) alphaMax = meterable;
                        }
                        // MUST be the number the adapter multiplies the command by, so read it from the
                        // ship rather than a field a caller may not have set.
                        double rateCapIP = Ship.GyroRateCap;
                        if (!(rateCapIP > 1e-6)) rateCapIP = CommandedRateCapBase;

                        // Work relative to the setpoint's own motion, so tracking a moving target needs
                        // no extra term.
                        Vector3D omegaRel = omegaBody - Vector3D.Transform(_setpointRateWorld, invOrient);
                        double omegaU = Vector3D.Dot(omegaRel, u);   // closing rate about u
                        double v = -omegaU;                          // d(theta)/dt

                        // BRAKE ON LESS THAN YOU COMMAND. The block above already says "brake on the
                        // CONSERVATIVE alpha (early is safe), command the GENEROUS one" -- but both
                        // were alphaMax, so there was no margin at all and ANY shortfall in delivered
                        // braking becomes overshoot with no way to recover. Measured post-fix: the
                        // ship still reached the setpoint at 0.42 rad/s of closing rate where its own
                        // curve allowed 0.066, i.e. 8x over.
                        //
                        // Under-braking and over-braking are NOT symmetric for a law converging on a
                        // target: braking early means arriving slightly short with rate to spare,
                        // which the sBand null-branch cleans up in a tick or two; braking late means
                        // sailing through and coming round again. So bias the switch, not the command.
                        double alphaSwitch = alphaMax * SwitchAlphaMargin;
                        if (alphaSwitch < 1e-3) alphaSwitch = 1e-3;
                        double s = angleRad
                                 + Math.Sign(v) * StopAngle(v, alphaSwitch, SlowdownRatePerSec)
                                 + v * h * 0.5;
                        // One control period's uncertainty in s. Inside that band the sign of s is not
                        // knowable from sampled state; null the residual rate instead of banging on a
                        // coin-flip. Derived band, not a tuned deadband.
                        double sBand = Math.Abs(v) * h * 0.5
                                     + 0.5 * alphaSwitch * h * h;
                        // PROPORTIONAL ENDGAME. Bang-bang is time-optimal only when the delivered
                        // acceleration matches the modelled one; where it does not, the law chatters
                        // at the setpoint because every tick commands FULL authority in one direction
                        // or the other. It does not match here: SE's free slowdown adds an unmodelled
                        // ~2.1*omega of extra braking, so the ship stops short of the switching curve,
                        // re-accelerates, and hunts. Measured on MUNDOZER: the gyro command saturated
                        // 20-37% of the time while merely holding, and a hand-flown 180 on the same
                        // hull rode a smooth ramp to 2.56 rad/s and eased to zero without saturating
                        // once. The deadbeat sBand branch (aU = v/h) is bang-bang too -- it asks to
                        // null the residual rate in ONE period, which saturates for any |v| above
                        // alphaCmd*h (0.042 rad/s here).
                        //
                        // So inside the hand-off angle, track a critically damped second order
                        // response instead: theta'' = -(2/tau) theta' - (1/tau^2) theta. With
                        // dv/dt = -aU that is aU = (2/tau) v + theta/tau^2, still expressed as an
                        // acceleration so the rate encoding downstream is unchanged (the legacy PD
                        // branch below emits a TORQUE fraction, which the plugin adapter would
                        // misread as a rate -- do not hand off to it).
                        //
                        // tau is derived, not tuned: the min-time time-to-stop from the hand-off
                        // angle, so the endgame runs at the same natural pace as the slew it follows.
                        double handoffRad = HandoffAngleDeg * Math.PI / 180.0;
                        double aU;
                        if (angleRad < handoffRad && handoffRad > 1e-6)
                        {
                            double tau = Math.Sqrt(2.0 * handoffRad / Math.Max(1e-6, alphaCmd));
                            if (tau < 4.0 * h) tau = 4.0 * h;      // never faster than the loop can act
                            aU = (2.0 / tau) * v + angleRad / (tau * tau);
                        }
                        else
                        {
                            aU = Math.Abs(s) < sBand
                                ? v / h                             // kill the residual rate this period
                                : (s > 0.0 ? alphaCmd : -alphaCmd); // otherwise full authority, correct sign
                        }
                        if (aU > alphaCmd) aU = alphaCmd; else if (aU < -alphaCmd) aU = -alphaCmd;
                        // Hardware rate ceiling: coast there rather than command past it.
                        if (aU > 0.0 && omegaU >= rateCapIP) aU = 0.0;

                        // Rotation off the closing axis does no useful work, so it gets damped -- but
                        // BOUNDED, not deadbeat.
                        //
                        // A one-tick kill asks for |omegaPerp|/h, i.e. 60x the perpendicular rate. Any
                        // perpendicular rate above alphaCmd*h -- 0.0147 rad/s on this hull -- therefore
                        // exceeds the ENTIRE plant authority on its own, and the joint clamp below then
                        // scales the along-axis term down by the same ratio. Measured live: a 0.2 rad/s
                        // roll made |aDes| 12.0 against alphaCmd 0.88, leaving the closing axis 0.064
                        // rad/s^2 -- 7% of the gyro -- so a 180 slew braked at 0.15-0.23 instead of
                        // 0.88 and sailed 35 deg past the setpoint. It is self-sustaining: the deadbeat
                        // cannot kill the roll in one tick either, so the starvation never lifts, and
                        // the alternating full-authority command limit-cycles at ~1.4 Hz.
                        //
                        // The original note -- "giving the along-axis first claim leaves the
                        // perpendicular nothing, and the ship tumbles about an undamped axis" -- is a
                        // real failure mode, so the perpendicular keeps a guaranteed share rather than
                        // a leftover. A share is enough because damping does not need to be deadbeat:
                        // PerpAuthorityFrac of 0.88 kills 0.2 rad/s in ~0.6 s while the aim keeps
                        // the rest.
                        // SIZE THE DAMPER ON THE AXIS IT ACTUALLY TORQUES ABOUT. alphaCmd is derived
                        // from iEff along the ERROR direction, but the rate being killed here is
                        // perpendicular to it -- a different axis, and on an anisotropic hull a wildly
                        // different plant. MUNDOZER: inertia 6.33e8 pitch / 6.30e8 yaw / 2.05e7 roll,
                        // so alpha is 11 about the aim and 341 about roll, a 31:1 spread. Budgeting the
                        // roll damper out of the pitch/yaw plant asks for a fraction of what the hull
                        // can actually deliver on that axis.
                        // MEASURED, 60 Hz, SUNDIAL HAULER (alpha 0.714/0.729/3.257, roll = grid Z):
                        // aReq.Z sat flat at +1.140 for six ticks, crossed zero in four, sat flat at
                        // -1.138 for six, while om.Z rang -0.128 -> +0.151 -> back. Period ~0.57 s
                        // (1.75 Hz), amplitude +/-0.14 rad/s. 1.140 is exactly PerpAuthorityFrac *
                        // alpha_roll, i.e. the cap, so this damper is saturated essentially always.
                        //
                        // A settle time was tried here and REVERTED. Deriving it from the file's own
                        // "never faster than the loop can act" rule gives max(4h, |w|/perpCap) =
                        // 0.123 s, whose demand is 1.14 -- the cap again. Nulling a rate in minimum
                        // time IS bang-bang, so escaping saturation needs a free factor, and this
                        // controller does not take free factors.
                        //
                        // It is also the wrong suspect. Clamped, this changes the rate by alpha*h =
                        // 1.14/60 = 0.019 rad/s per tick, so it converges to +/-0.019 and stays. The
                        // measured ring is +/-0.14, SEVEN TIMES its own resolution -- a damper cannot
                        // sustain an oscillation that much larger than one step. Something re-injects
                        // roll rate each cycle; find that, do not detune this.
                        Vector3D omegaPerp = omegaRel - omegaU * u;
                        Vector3D aPerpDes = -omegaPerp * (1.0 / h);
                        double perpCap = alphaCmd * PerpAuthorityFrac;
                        double perpLen = omegaPerp.Length();
                        if (perpLen > 1e-9)
                        {
                            Vector3D pDir = omegaPerp / perpLen;
                            double iPerp = pDir.X * pDir.X * I.X
                                         + pDir.Y * pDir.Y * I.Y
                                         + pDir.Z * pDir.Z * I.Z;
                            if (iPerp < 1.0) iPerp = Math.Max(1.0, Math.Min(I.X, Math.Min(I.Y, I.Z)));
                            double alphaPerpAvail = Ship.MaxTorque / iPerp;
                            // Same meterability bound the slew uses: authority a 60 Hz loop cannot
                            // resolve is spent overshooting, damper or not.
                            if (MinSlewSeconds > 1e-3)
                            {
                                double mPerp = Ship.GyroRateCap / MinSlewSeconds;
                                if (mPerp > 1e-6 && alphaPerpAvail > mPerp) alphaPerpAvail = mPerp;
                            }
                            double capOwn = alphaPerpAvail * PerpAuthorityFrac;
                            if (capOwn > perpCap) perpCap = capOwn;

                            // GROWTH FLOOR. A fixed share assumes off-axis rate is incidental --
                            // leakage the damper mops up at its leisure. On these hulls it is not.
                            //
                            // SUNDIAL: I_z 5.70e8 < I_y 2.548e9 < I_x 2.602e9, so a yaw slew turns
                            // about the INTERMEDIATE principal axis, which is dynamically unstable
                            // (tennis-racket). X and Y differ by 2%, so nearly any slew that is not
                            // about roll lands on or beside it; MUNDOZER is the same shape. Measured
                            // at 60 Hz: om.X and om.Z ringing together at ~2 Hz, 90 deg apart, while
                            // om.Y ramped smoothly -- coning, generated by the plant, not leakage.
                            //
                            // Against a divergence, a minority share is not a budget, it is a losing
                            // race: the damper asked 3.5-6.6 rad/s^2 and was handed 0.26-1.14.
                            //
                            // A GROWTH FLOOR WAS TRIED HERE AND REMOVED: it never fired. Measured on
                            // SUNDIAL, |omega_perp| grows ~0.095 rad/s over ~15 ticks = 0.38 rad/s^2
                            // against a perpCap of 1.14, so the damper already has 3x the authority
                            // needed to arrest the coning and rings anyway.
                            //
                            // That kills the starvation story. Authority is NOT the constraint here,
                            // and neither is lag: the latency probe answers in one tick (17 ms) with a
                            // monotone ramp and a plant model good to 3.4%. Accurate plant, no dead
                            // time, ample authority, correct sign -- and a stable +/-0.15 rad/s ring
                            // at ~2 Hz that a clamped deadbeat should hold to +/-alpha*h = +/-0.019.
                            // The residual is 7x unexplained. Measure the perpendicular channel over a
                            // full cycle before theorising again.
                        }
                        // Outside the guard too: a tick where perpLen underflows must not leave a
                        // stale previous value to differentiate against on the next one.
                        _prevPerpMag = perpLen;
                        double perpMag = aPerpDes.Length();
                        DbgPerpRaw = perpMag;
                        if (perpMag > perpCap && perpMag > 1e-9) aPerpDes *= perpCap / perpMag;
                        DbgPerpCapped = aPerpDes.Length();
                        Vector3D aDes = u * aU + aPerpDes;
                        // JOINT CLAMP ON THE COMPOSITE DIRECTION, for the same reason: aDes mixes the
                        // aim axis and the damped axis, so the ceiling that applies to it is the one
                        // for ITS direction, not the aim's. Clamping a mostly-roll demand against the
                        // pitch/yaw ceiling threw away authority the hull had.
                        double aMag = aDes.Length();
                        if (aMag > 1e-9)
                        {
                            Vector3D aDir = aDes / aMag;
                            double iDes = aDir.X * aDir.X * I.X
                                        + aDir.Y * aDir.Y * I.Y
                                        + aDir.Z * aDir.Z * I.Z;
                            if (iDes < 1.0) iDes = Math.Max(1.0, Math.Min(I.X, Math.Min(I.Y, I.Z)));
                            double aCeil = Ship.MaxTorque / iDes;
                            if (MinSlewSeconds > 1e-3)
                            {
                                double mDes = Ship.GyroRateCap / MinSlewSeconds;
                                if (mDes > 1e-6 && aCeil > mDes) aCeil = mDes;
                            }
                            if (aCeil < alphaCmd) aCeil = alphaCmd;   // never below the aim's own budget
                            if (aMag > aCeil) aDes *= aCeil / aMag;
                        }
                        // What we ASKED for, so next tick's observer can tell a weak plant from an
                        // idle one. See the decay gate in the alphaObserved block.
                        _lastDemand = aDes.Length();
                        // Invert the servo: the gap that yields exactly aDes.
                        Vector3D omegaCmd = omegaBody + AccelToRateGap(aDes);
                        // Back to command space (anti-parallel, normalised by the rate cap).
                        control = -omegaCmd / rateCapIP;
                        if (TraceMinTime != null)
                            TraceMinTime(new double[] {
                                angleRad, aU, aDes.Length(), AccelToRateGap(aDes).Length(),
                                omegaBody.Length(), v, s, sBand, alphaSwitch, alphaCmd });
                    }
                    else
                    {
                        // PLANT SIGN: the gyro stores (0f - value) and servos toward (target - omega), so
                        // omega is driven ANTI-PARALLEL to the command. This < 0 test and the +Kd below
                        // encode ONE convention and must move together.
                        bool rotatingToward = Vector3D.Dot(omegaBody, errorDir) < 0.0;
                        if (!rotatingToward)
                            control = errorDir * 0.95;
                        else if (brakingAngle >= angleRad * 0.85)
                            control = -errorDir;
                        else
                            control = errorDir * 0.95;
                    }
                }
                else
                {
                    // PD (critically damped at default Kp=1, Kd=2).
                    control = Kp * errorBody + Kd * omegaBody;
                }

                // ---- roll closure about the forward axis ----
                // Superimposed on whichever law ran above, but only once the aim is close: rolling
                // during a big slew spends gyro budget fighting the flip. The command convention is
                // anti-parallel (see the plant sign notes), so the written component is -omega/cap.
                if (RollRefWorld.HasValue && angleRad < 0.5)
                {
                    Vector3D fwdW = currentFwd;
                    Vector3D wantUp = RollRefWorld.Value - fwdW * Vector3D.Dot(RollRefWorld.Value, fwdW);
                    Vector3D upW = Vector3D.Transform(Vector3D.Up, rb.Orientation);
                    Vector3D haveUp = upW - fwdW * Vector3D.Dot(upW, fwdW);
                    if (wantUp.LengthSquared() > 1e-9 && haveUp.LengthSquared() > 1e-9)
                    {
                        wantUp = Vector3D.Normalize(wantUp);
                        haveUp = Vector3D.Normalize(haveUp);
                        double rollAng = Math.Atan2(
                            Vector3D.Dot(Vector3D.Cross(haveUp, wantUp), fwdW),
                            Vector3D.Dot(haveUp, wantUp));
                        if (Math.Abs(rollAng) > RollDeadbandRad)
                        {
                            double wRoll = RollGain * rollAng;
                            if (wRoll > RollRateCap) wRoll = RollRateCap;
                            else if (wRoll < -RollRateCap) wRoll = -RollRateCap;
                            double capR = CommandRateCap > 1e-6 ? CommandRateCap : 1.0;
                            Vector3D fwdBody = Vector3D.Transform(fwdW, invOrient);
                            control -= fwdBody * (wRoll / capR);
                            // THE SUSPECT. This is the one channel that does not go through aDes and
                            // the authority allocation -- it injects a rate straight into a finished
                            // command. If roll rings while this is non-zero and settles while it is
                            // zero, that is the excitation.
                            DbgRollCmd = wRoll / capR;
                        }
                    }
                }

                // SATURATE BY THE NORM, NOT PER AXIS. The setters below Clamp1 each component on its
                // own, so any |control| > 1 does not merely shorten the command -- it TILTS it. A
                // command along (0.10, 0.90, 0.42) scaled to |control| = 4.3 lands as
                // (0.72, 1.00, 1.00) normalised: a different rotation axis, and a different one every
                // tick as omegaBody moves. Scaling here keeps the axis and lets the clamp be a no-op.
                //
                // This is the same defect PluginShip.DiscNorm fixes on the torque vector, but that
                // sits DOWNSTREAM of these setters and never sees the command that was tilted here.
                // Still needed with ServoRampFrames off: omegaCmd = omegaBody + dw, and |omegaBody|
                // reaches the 3.293 rad/s Havok cap against a rateCapIP of ~3.456.
                double cMag = Math.Max(Math.Abs(control.X),
                              Math.Max(Math.Abs(control.Y), Math.Abs(control.Z)));
                if (cMag > 1.0) control /= cMag;

                // Clamp to [-1,1], then deadband (after clamp, so the band is in command units).
                double scale = RollDeadbandScale;
                if (scale < 0.0) scale = 0.0; else if (scale > 1.0) scale = 1.0;
                double effectiveRollDeadband = DeadbandRoll * scale;
                // The rate-command deadband does not apply to the inverse-plant law.
                if (InversePlant) { DeadbandPitchEff = 0.0; DeadbandYawEff = 0.0; effectiveRollDeadband = 0.0; }
                else { DeadbandPitchEff = DeadbandPitch; DeadbandYawEff = DeadbandYaw; }
                Ship.Pitch = ApplyDeadband(MathHelpers.Clamp1(control.X), DeadbandPitchEff);
                Ship.Yaw = ApplyDeadband(MathHelpers.Clamp1(control.Y), DeadbandYawEff);
                Ship.Roll = ApplyDeadband(MathHelpers.Clamp1(control.Z), effectiveRollDeadband);
            }

            // ---- 60 Hz diagnostics, written every Update, read by the adapter's cttrace ----
            // Static so the adapter can reach them without a handle on the controller.
            public static double DbgAngleRad;      // nose error this tick
            public static double DbgRollCmd;       // roll-closure injection into control, 0 if it did not run
            public static double DbgPerpRaw;       // |aPerpDes| the deadbeat asked for
            public static double DbgPerpCapped;    // |aPerpDes| after PerpAuthorityFrac

            // Last measured nose error, for status reporting (the mod exposes this as AutoRotate_Angle).
            public double LastAngleRad { get; private set; }
            // The max-hold acceleration estimate, exposed because it sizes the braking curve in BOTH
            // laws: inflate it and the controller believes it can stop later than it can.
            public double LastAlphaObserved { get { return _alphaObserved; } }
            // Which law produced the last command, and the rate ceiling it allowed.
            public string LastLaw = "none";
            public double LastRateCeiling;
        }
    }
}

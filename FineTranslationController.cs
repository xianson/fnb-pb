using System;
using VRageMath;

namespace IngameScript
{
    partial class Program
    {
        // Point-to-point translation: flip-aware bang-bang (Burn/Flip/Brake) plus a flip-free
        // Translate and a heavy-gravity creep. Verbatim port of the mod's
        // WaypointQueue/FineTranslationController.cs.
        //
        // In the mod this runs only as the last ~80 m of a spline leg. Here it runs the WHOLE A->B
        // leg: on a straight 2-control-point path the spline follower's baked schedule degenerates
        // to exactly this bang-bang, and SolveHopPeakSpeed already solves the accel+flip+brake
        // triangle in closed form. See the README for the full justification.
        public sealed class FineTranslationController
        {
            public enum Phase { Burn, Flip, Brake, GravityCreep, Translate, Done }

            public static string PhaseName(Phase p)
            {
                switch (p)
                {
                    case Phase.Burn: return "burn";
                    case Phase.Flip: return "flip";
                    case Phase.Brake: return "brake";
                    case Phase.GravityCreep: return "creep";
                    case Phase.Translate: return "translate";
                    case Phase.Done: return "done";
                }
                return "?";
            }
            public bool IsDone { get { return CurrentPhase == Phase.Done; } }

            public readonly IShip Ship;
            public readonly OrientationController Attitude;

            public Vector3D StartPosition;
            public Vector3D Target;
            public Phase CurrentPhase = Phase.Burn;

            // Tilt PD (points the main drive at the cross-track residual).
            public double CrossPosGain = 0.5;
            public double CrossVelGain = 1.4;
            public double MaxTiltAngle = Math.PI / 6.0;

            // Lateral RCS PD.
            public double LateralPosGain = 1.0;
            public double LateralVelGain = 2.0;

            // Release to SE dampeners only below this speed AND inside the stop budget.
            public double DampenerHandoffSpeed = 25.0;
            public double DampenerStopBudgetM = 50.0;

            // Wall-clock bail for the whole terminal path.
            public double Timeout = 120.0;
            public double TimeoutFloorSeconds = 120.0;
            public double TimeoutPlanMult = 4.0;

            // Speed below which a within-tolerance ship is treated as already arrived.
            public double AlreadyArrivedSpeed = 2.0;

            // Arrival tolerance on TOTAL distance to Target (along + cross).
            // Instances are built per sub-mission, so the CustomData knob has to reach them through a
            // static default rather than being set on one of them.
            public static double DefaultArrivalDistM = 40.0;
            public double ArrivalDistM = DefaultArrivalDistM;
            // Let-go radius for the already-arrived fast-path.
            public double FastArriveDistM = 25.0;

            // Re-base bound.
            public int MaxRebases = 3;
            int _rebaseCount;
            public double RebaseMinProgressM = 2.0;
            double _distAtLastRebase = double.PositiveInfinity;

            // ---- Hop-length-aware speed profile (flip-aware bang-bang) ----
            public double HopMargin = 0.8;        // 20% headroom for the brake
            public double HopBrakeMargin = 0.85;  // aBrakeEff = aMax * 0.85 (delivery)
            public double GravityCreepSpeed = 10.0;
            public double HopPeakSpeedFloor = 2.0;
            double _hopPeakSpeed;
            // The cap only LIMITS acceleration -- Burn has no brake authority, so it cannot bleed
            // off carried speed.
            bool _hopCapArmed;

            // Alignment at which the main drive may fire, and therefore also where Flip is done.
            // One gate, so the two cannot drift apart.
            public double BrakeThrustGate = 0.15;

            public double FlipLeadTime;

            public double Elapsed { get; private set; }
            public double TopSpeed { get; private set; }
            public double ArrivalTime { get; private set; }
            public double ArrivalAlongTrackError { get; private set; }
            public double ArrivalCrossTrackError { get; private set; }
            public double ArrivalAlongTrackVel { get; private set; }
            public double ArrivalCrossTrackVel { get; private set; }

            Vector3D _lineDir;

            public FineTranslationController(IShip ship, OrientationController attitude, Vector3D target)
            {
                Ship = ship;
                Attitude = attitude;
                Target = target;
                ArrivalTime = -1.0;
                StartPosition = ship.Body.Position;

                Vector3D axis = Target - StartPosition;
                _lineDir = axis.LengthSquared() > 1e-12 ? Vector3D.Normalize(axis) : Vector3D.UnitZ;

                FlipLeadTime = OrientationController.DeriveFlipLeadTime(ship);

                _hopPeakSpeed = SolveHopPeakSpeed(axis.Length());
                ArmHopCap();

                // Maneuver select. Within CloseTranslateM always translate; beyond it, flip-hop vs
                // translate by time estimate.
                double tTranslate = EstimateTranslateTime(axis);
                double tHop = EstimateFlipHopTime(axis.Length());
                if (axis.Length() < CloseTranslateM || tTranslate <= tHop)
                    EnterTranslate(StartPosition);

                double tPlanned = Math.Min(tTranslate, tHop);
                if (double.IsNaN(tPlanned) || double.IsInfinity(tPlanned)) tPlanned = 0.0;
                Timeout = Math.Max(TimeoutFloorSeconds,
                                   TimeoutPlanMult * (MaxRebases + 1) * (tPlanned + FlipLeadTime));
            }

            // Flip attitude frozen at entry: re-aiming at LIVE retrograde every tick makes the flip
            // chase a target its own residual motion keeps moving, so it arrives hot.
            Vector3D _flipDir;
            bool _flipDirValid;

            void EnterFlip(Vector3D desired)
            {
                _flipDir = desired.LengthSquared() > 1e-12 ? Vector3D.Normalize(desired) : -_lineDir;
                _flipDirValid = true;
                CurrentPhase = Phase.Flip;
            }

            // ---- Flip-free Translate machinery ----
            Quaternion _holdOrientation;
            bool _holdValid;
            public double TranslateVelGain = 1.5;
            public double TranslateBrakeMargin = 0.8;
            // Within this range of the waypoint, always translate (flip-free) instead of flip-hopping:
            // a hop over the terminal stretch wastes the flip reserve and arrives hot. 2 km covers the
            // whole fine-handoff range, so the terminal approach is translated end to end.
            public double CloseTranslateM = 2000.0;

            void EnterTranslate(Vector3D pos)
            {
                Vector3D axis = Target - pos;
                if (axis.LengthSquared() > 1e-12) _lineDir = Vector3D.Normalize(axis);
                _holdOrientation = Ship.Body.Orientation;
                _holdValid = true;
                CurrentPhase = Phase.Translate;
            }

            // Per-axis bang-bang time for a flip-free translate over the body-frame offset of 'axis'.
            double EstimateTranslateTime(Vector3D axis)
            {
                IRigidBody rb = Ship.Body;
                Vector3D eB = Vector3D.Transform(axis, Quaternion.Conjugate(rb.Orientation));
                double inv = rb.InvMass;
                double t = 0.0;
                t = Math.Max(t, AxisTime(-eB.Z, Ship.MaxForwardThrust * inv, Ship.MaxBackwardThrust * inv));
                t = Math.Max(t, AxisTime(eB.X, Ship.MaxRightThrust * inv, Ship.MaxLeftThrust * inv));
                t = Math.Max(t, AxisTime(eB.Y, Ship.MaxUpThrust * inv, Ship.MaxDownThrust * inv));
                return t;
            }

            public double EstimateSecondsRemaining()
            {
                if (CurrentPhase == Phase.Done) return 0.0;
                Vector3D axis = Target - Ship.Body.Position;
                double L = axis.Length();
                // The closer stops working at ArrivalDistM, not at the target.
                double Lfly = L - ArrivalDistM;
                if (Lfly <= 1e-3) return 0.0;
                axis *= Lfly / L;
                L = Lfly;
                if (CurrentPhase == Phase.GravityCreep)
                    return L / Math.Max(1e-3, GravityCreepSpeed);
                double t = EstimateTranslateTime(axis);
                double hop = EstimateFlipHopTime(L);
                if (hop < t) t = hop;
                if (t > 1e8) t = Math.Max(0.0, Timeout - Elapsed);
                if (t < 0.0) t = 0.0;
                return t;
            }

            double AxisTime(double e, double aPos, double aNeg)
            {
                double d = Math.Abs(e);
                if (d < 1.0) return 0.0;
                double aAcc = e >= 0.0 ? aPos : aNeg;   // accelerate toward target
                double aBrk = e >= 0.0 ? aNeg : aPos;   // stop that motion
                if (aAcc < 1e-3 || aBrk < 1e-3) return 1e9;
                double aH = aAcc * aBrk / (aAcc + aBrk) * TranslateBrakeMargin;
                double vPk = Math.Sqrt(2.0 * d * aH);
                return vPk / aAcc + vPk / aBrk;
            }

            // Flip-hop time: accel to the hop peak + flip coast + retro brake.
            double EstimateFlipHopTime(double L)
            {
                IRigidBody rb = Ship.Body;
                double aMax = Ship.MaxForwardThrust * rb.InvMass;
                if (aMax < 1e-6) return 1e9;
                double v = SolveHopPeakSpeed(L);
                return v / aMax + 1.5 * FlipLeadTime + v / (aMax * HopBrakeMargin);
            }

            // One per-axis Translate command.
            double TranslateAxisCmd(double e, double vAxis, double gAxis, double aPos, double aNeg)
            {
                double aBrk = e >= 0.0 ? aNeg : aPos;       // budget that stops motion toward target
                double vDes = 0.0;
                double d = Math.Abs(e);
                if (d > 0.5 && aBrk > 1e-3)
                    vDes = Math.Sign(e) * Math.Sqrt(2.0 * aBrk * TranslateBrakeMargin * d);
                double aDes = TranslateVelGain * (vDes - vAxis) - gAxis;
                double budget = aDes >= 0.0 ? aPos : aNeg;
                if (budget < 1e-6) return 0.0;
                double u = aDes / budget;
                if (u > 1.0) u = 1.0; else if (u < -1.0) u = -1.0;
                return u;
            }

            // Heavy gravity: |g| exceeds the weakest perpendicular RCS authority.
            bool IsHeavyGravity()
            {
                IRigidBody rb = Ship.Body;
                double gMag = Ship.Gravity.Length();
                if (gMag < 1e-6) return false;
                double aUp = Ship.MaxUpThrust * rb.InvMass;
                double aDown = Ship.MaxDownThrust * rb.InvMass;
                double aLeft = Ship.MaxLeftThrust * rb.InvMass;
                double aRight = Ship.MaxRightThrust * rb.InvMass;
                double aPerpMin = Math.Min(Math.Min(aUp, aDown), Math.Min(aLeft, aRight));
                return gMag > aPerpMin;
            }

            void ArmHopCap()
            {
                double speed = Ship.Body.LinearVelocity.Length();
                _hopCapArmed = _hopPeakSpeed > speed;
            }

            // Flip-aware, gravity-aware bang-bang peak speed for a hop of length L.
            double SolveHopPeakSpeed(double L)
            {
                IRigidBody rb = Ship.Body;
                double aMax = Ship.MaxForwardThrust * rb.InvMass;
                if (aMax < 1e-6 || L <= 0.0) return HopPeakSpeedFloor;
                double aBrakeEff = aMax * HopBrakeMargin;
                double tFlip = FlipLeadTime;
                double gAlong = Vector3D.Dot(Ship.Gravity, _lineDir);
                // Gravity along the hop reduces brake decel (only the toward-target component hurts).
                double aBrakeEffG = aBrakeEff - Math.Max(0.0, gAlong);
                if (aBrakeEffG < 0.1 * aBrakeEff) aBrakeEffG = 0.1 * aBrakeEff;
                double k = 0.5 / aMax + 0.5 / aBrakeEffG;
                double b = tFlip + gAlong * tFlip / aBrakeEffG;
                double gt = gAlong * tFlip;                       // coast-exit speed gain
                double c = 0.5 * gAlong * tFlip * tFlip
                         + gt * gt / (2.0 * aBrakeEffG)
                         - HopMargin * L;
                double disc = b * b - 4.0 * k * c;
                if (disc < 0.0) disc = 0.0;
                double v = (-b + Math.Sqrt(disc)) / (2.0 * k);
                if (v < HopPeakSpeedFloor) v = HopPeakSpeedFloor;
                return v;
            }

            public void Update(double dt)
            {
                IRigidBody rb = Ship.Body;
                Vector3D pos = rb.Position;
                Vector3D v = rb.LinearVelocity;
                double speed = v.Length();
                if (speed > TopSpeed) TopSpeed = speed;

                double physSpeed = rb.PhysicalLinearVelocity.Length();
                double gateSpeed = Math.Max(speed, physSpeed);
                double remainingAlongGate = Vector3D.Dot(Target - pos, _lineDir);
                double distToTarget = (Target - pos).Length();

                // Live rule: once inside CloseTranslateM, translate -- switch out of any hop phase
                // (Burn/Flip/Brake) into a direct flip-free translate for the terminal approach.
                if (CurrentPhase != Phase.Done && CurrentPhase != Phase.Translate
                    && CurrentPhase != Phase.GravityCreep && distToTarget < CloseTranslateM)
                {
                    EnterTranslate(pos);
                }

                double aBrakeDamp = Ship.MaxForwardThrust * rb.InvMass;
                double dampStopDist = aBrakeDamp > 1e-6
                    ? (gateSpeed * gateSpeed) / (2.0 * aBrakeDamp)
                    : double.PositiveInfinity;
                bool slow = gateSpeed < DampenerHandoffSpeed
                                 && dampStopDist <= DampenerStopBudgetM;
                bool planePast = remainingAlongGate <= 0.0;

                if (CurrentPhase != Phase.Done && Elapsed > Timeout)
                {
                    CurrentPhase = Phase.Done;
                    RecordArrival();
                }

                // ---- Already-arrived fast-paths (plane-independent) ----
                bool heavyG = IsHeavyGravity();
                bool tightArrived = slow && distToTarget <= FastArriveDistM;
                bool atRestArrived = gateSpeed < AlreadyArrivedSpeed
                    && distToTarget <= ArrivalDistM;
                if (CurrentPhase != Phase.Done && (tightArrived || atRestArrived))
                {
                    CurrentPhase = Phase.Done;
                    RecordArrival();
                }
                if (CurrentPhase != Phase.Done && !heavyG && slow && planePast)
                {
                    if (distToTarget <= ArrivalDistM)
                    {
                        CurrentPhase = Phase.Done;
                        RecordArrival();
                    }
                    else
                    {
                        // Off-line miss. Re-base if attempts remain AND the last one made progress.
                        bool madeProgress = distToTarget < _distAtLastRebase - RebaseMinProgressM;
                        if (_rebaseCount < MaxRebases && madeProgress)
                        {
                            // Re-bases prefer flip-free, but Translate can only close an axis it can
                            // also BRAKE on; where it is infeasible, re-base into the hop instead.
                            StartPosition = pos;
                            Vector3D residual = Target - pos;
                            if (EstimateTranslateTime(residual) < 1e8)
                                EnterTranslate(pos);
                            else
                            {
                                if (residual.LengthSquared() > 1e-12) _lineDir = Vector3D.Normalize(residual);
                                _hopPeakSpeed = SolveHopPeakSpeed(residual.Length());
                                ArmHopCap();
                                _flipDirValid = false;
                                CurrentPhase = Phase.Burn;
                            }
                            _rebaseCount++;
                            _distAtLastRebase = distToTarget;
                        }
                        else
                        {
                            CurrentPhase = Phase.Done;
                            RecordArrival();
                        }
                    }
                }

                // Heavy gravity: a hop is structurally unsafe (unpowered flip free-falls off the line).
                if (CurrentPhase != Phase.Done
                    && CurrentPhase != Phase.GravityCreep
                    && heavyG)
                {
                    CurrentPhase = Phase.GravityCreep;
                }

                if (CurrentPhase == Phase.Done)
                {
                    Ship.ThrottleForward = 0.0;
                    Ship.StrafeRight = 0.0;
                    Ship.StrafeUp = 0.0;
                    Ship.Pitch = 0.0;
                    Ship.Yaw = 0.0;
                    Ship.Roll = 0.0;
                    Ship.DampenersWanted = true;
                    Elapsed += dt;
                    return;
                }

                Vector3D fromStart = pos - StartPosition;
                double alongPos = Vector3D.Dot(fromStart, _lineDir);
                Vector3D crossPos = fromStart - alongPos * _lineDir;
                double vAlong = Vector3D.Dot(v, _lineDir);
                Vector3D vCross = v - vAlong * _lineDir;

                double remainingAlong = Vector3D.Dot(Target - pos, _lineDir);
                double aMax = Ship.MaxForwardThrust * rb.InvMass;
                Vector3D gravity = Ship.Gravity;
                double brakeDistAlong = vAlong > 0.0 ? (vAlong * vAlong) / (2.0 * aMax) : 0.0;

                // Tilt PD (points the main drive).
                Vector3D aTiltDesired = -CrossPosGain * crossPos - CrossVelGain * vCross;
                double aTiltMax = aMax * Math.Sin(MaxTiltAngle);
                double aTiltMag = aTiltDesired.Length();
                if (aTiltMag > aTiltMax && aTiltMag > 1e-9)
                    aTiltDesired *= (aTiltMax / aTiltMag);

                Vector3D aLatWorld = -LateralPosGain * crossPos - LateralVelGain * vCross;

                Vector3D retro = speed > 1e-3 ? -v / speed : -_lineDir;
                bool fireLateral = false;

                // Dampeners OFF by default for every powered phase, re-asserted each tick.
                Ship.DampenersWanted = false;

                switch (CurrentPhase)
                {
                    case Phase.Burn:
                        {
                            double aAlong = aMax;
                            bool coasting = _hopCapArmed && vAlong >= _hopPeakSpeed;
                            if (coasting)
                                aAlong = 0.0;   // coast at the hop peak; flip trigger takes over
                            // Coasting drops the tilt term from the NOSE demand: with no along-track
                            // demand it is the only term left, so the nose would aim ~90 deg off the
                            // line and wander. Lateral RCS still corrects cross-track.
                            Vector3D aDesired = aAlong * _lineDir + (coasting ? Vector3D.Zero : aTiltDesired);
                            Vector3D noseDir; double throttleMag;
                            MathHelpers.GravityCompensatedCommand(aDesired, gravity, aMax,
                                _lineDir,
                                out noseDir, out throttleMag);
                            Attitude.Target = MathHelpers.LookAlong(noseDir);
                            double alignErrBurn = MathHelpers.AngleBetween(Ship.Forward, noseDir);
                            Ship.ThrottleForward = alignErrBurn <= BrakeThrustGate ? throttleMag : 0.0;
                            fireLateral = true;

                            double leadDist = Math.Max(0.0, vAlong) * FlipLeadTime;
                            if (brakeDistAlong + leadDist >= remainingAlong) EnterFlip(aMax * retro + aTiltDesired);
                            break;
                        }
                    case Phase.Flip:
                        {
                            Vector3D desired = _flipDirValid ? _flipDir : aMax * retro + aTiltDesired;
                            Attitude.Target = MathHelpers.LookAlong(desired);
                            // Hover thrust along nose once aligned.
                            double hoverMag = gravity.Length();
                            double alignErrFlip = MathHelpers.AngleBetween(Ship.Forward, desired);
                            Ship.ThrottleForward = (hoverMag > 0.01 && alignErrFlip <= BrakeThrustGate)
                                ? Math.Min(1.0, hoverMag / aMax)
                                : 0.0;
                            if (alignErrFlip <= BrakeThrustGate)
                                CurrentPhase = Phase.Brake;
                            break;
                        }
                    case Phase.Brake:
                        {
                            if (remainingAlong <= 0.0)
                            {
                                if (speed > 1e-3)
                                {
                                    Vector3D brakeAxis = -v / speed;
                                    Attitude.Target = MathHelpers.LookAlong(brakeAxis);
                                    double alignErrOver = MathHelpers.AngleBetween(Ship.Forward, brakeAxis);
                                    Ship.ThrottleForward = alignErrOver <= BrakeThrustGate ? 1.0 : 0.0;
                                }
                                else
                                {
                                    Ship.ThrottleForward = 0.0;
                                    Ship.DampenersWanted = true;
                                }
                                break;
                            }
                            double brakeDecel;
                            if (vAlong > 0.0 && remainingAlong > 1e-3)
                                brakeDecel = Math.Min(aMax, (vAlong * vAlong) / (2.0 * remainingAlong));
                            else
                                brakeDecel = aMax;
                            Vector3D aDesired = brakeDecel * retro + aTiltDesired;
                            Vector3D noseDir; double throttleMag;
                            MathHelpers.GravityCompensatedCommand(aDesired, gravity, aMax,
                                retro,
                                out noseDir, out throttleMag);
                            Attitude.Target = MathHelpers.LookAlong(noseDir);
                            double alignErr = MathHelpers.AngleBetween(Ship.Forward, noseDir);
                            Ship.ThrottleForward = alignErr <= BrakeThrustGate ? throttleMag : 0.0;
                            fireLateral = true;
                            break;
                        }
                    case Phase.Translate:
                        {
                            // Flip-free residual mover: nose frozen at entry, three per-axis body loops.
                            if (_holdValid)
                                Attitude.Target = _holdOrientation;
                            Quaternion w2b = Quaternion.Conjugate(rb.Orientation);
                            Vector3D eB = Vector3D.Transform(Target - pos, w2b);
                            Vector3D vB = Vector3D.Transform(v, w2b);
                            Vector3D gB = Vector3D.Transform(gravity, w2b);
                            double inv = rb.InvMass;
                            Ship.ThrottleForward = TranslateAxisCmd(
                                -eB.Z, -vB.Z, -gB.Z,
                                Ship.MaxForwardThrust * inv, Ship.MaxBackwardThrust * inv);
                            Ship.StrafeRight = TranslateAxisCmd(
                                eB.X, vB.X, gB.X,
                                Ship.MaxRightThrust * inv, Ship.MaxLeftThrust * inv);
                            Ship.StrafeUp = TranslateAxisCmd(
                                eB.Y, vB.Y, gB.Y,
                                Ship.MaxUpThrust * inv, Ship.MaxDownThrust * inv);
                            Attitude.Update();
                            Elapsed += dt;
                            return;
                        }
                    case Phase.GravityCreep:
                        {
                            // Heavy gravity: any unpowered window free-falls off the line, so the main
                            // drive stays lit and the nose stays anti-gravity.
                            double gMag = gravity.Length();
                            Vector3D noseUp = gMag > 1e-6 ? -gravity / gMag : -_lineDir;
                            double gAlongCreep = Vector3D.Dot(gravity, _lineDir);
                            const double creepApproachTau = 2.0;
                            double vCap = Math.Max(0.0, remainingAlong) / creepApproachTau;
                            if (vCap > GravityCreepSpeed) vCap = GravityCreepSpeed;
                            const double creepTau = 0.4;
                            double aAlongCreep = (vCap - vAlong) / creepTau;
                            double noseDotLine = Vector3D.Dot(noseUp, _lineDir);
                            double throttleMagC;
                            if (Math.Abs(noseDotLine) > 1e-3)
                                throttleMagC = (aAlongCreep - gAlongCreep) / (aMax * noseDotLine);
                            else
                                throttleMagC = aMax > 1e-6 ? gMag / aMax : 0.0; // g perp to line: hover
                            if (throttleMagC < 0.0) throttleMagC = 0.0;
                            if (throttleMagC > 1.0) throttleMagC = 1.0;
                            Attitude.Target = MathHelpers.LookAlong(noseUp);
                            double alignErrCreep = MathHelpers.AngleBetween(Ship.Forward, noseUp);
                            // Keep the drive lit through gyro wobble, scaled by max(0, cos(align)).
                            double cosAlign = Math.Cos(alignErrCreep);
                            bool creepAligned = cosAlign > 0.0; // nose within 90 deg of up
                            Ship.ThrottleForward = creepAligned ? throttleMagC * cosAlign : 0.0;
                            fireLateral = creepAligned;
                            break;
                        }
                }

                double aRight = Ship.MaxRightThrust * rb.InvMass;
                double aLeft = Ship.MaxLeftThrust * rb.InvMass;
                double aUp = Ship.MaxUpThrust * rb.InvMass;
                double aDown = Ship.MaxDownThrust * rb.InvMass;
                // Gate on the SAME per-sign budgets the body uses, not on a min-collapsed scalar.
                bool haveLateral = aRight > 0.0 || aLeft > 0.0 || aUp > 0.0 || aDown > 0.0;
                if (fireLateral && haveLateral)
                {
                    Vector3D aLatBody = Vector3D.Transform(aLatWorld, Quaternion.Conjugate(rb.Orientation));
                    Ship.StrafeRight = MathHelpers.StrafeCmd(aLatBody.X, aRight, aLeft);
                    Ship.StrafeUp = MathHelpers.StrafeCmd(aLatBody.Y, aUp, aDown);
                }
                else
                {
                    Ship.StrafeRight = 0.0;
                    Ship.StrafeUp = 0.0;
                }

                Attitude.Update();
                Elapsed += dt;
            }

            // PB ADDITION, read-only, no effect on control: powered-burn seconds the current plan still
            // needs, for the fuel guard. Coast and flip are unpowered, so this is well under the ETA.
            public double RequiredBurnSeconds()
            {
                IRigidBody rb = Ship.Body;
                double aMax = Ship.MaxForwardThrust * rb.InvMass;
                if (aMax < 1e-6) return -1.0;
                if (CurrentPhase == Phase.Done) return 0.0;
                // These two are powered throughout, so their whole remaining time is burn time.
                if (CurrentPhase == Phase.Translate || CurrentPhase == Phase.GravityCreep)
                    return EstimateSecondsRemaining();

                double L = (Target - rb.Position).Length();
                double vNow = Vector3D.Dot(rb.LinearVelocity, _lineDir);
                if (vNow < 0.0) vNow = 0.0;
                double vPk = SolveHopPeakSpeed(L);
                if (vPk < vNow) vPk = vNow;
                double tAccel = CurrentPhase == Phase.Burn ? (vPk - vNow) / aMax : 0.0;
                if (tAccel < 0.0) tAccel = 0.0;
                double tBrake = vPk / (aMax * HopBrakeMargin);
                return tAccel + tBrake;
            }

            void RecordArrival()
            {
                ArrivalTime = Elapsed;
                Vector3D ePos = Ship.Body.Position - Target;
                ArrivalAlongTrackError = Vector3D.Dot(ePos, _lineDir);
                ArrivalCrossTrackError = (ePos - ArrivalAlongTrackError * _lineDir).Length();
                Vector3D v = Ship.Body.LinearVelocity;
                ArrivalAlongTrackVel = Vector3D.Dot(v, _lineDir);
                ArrivalCrossTrackVel = (v - ArrivalAlongTrackVel * _lineDir).Length();
            }
        }
    }
}

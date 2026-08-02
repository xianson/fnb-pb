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
            public static double AlphaObservedDecay = 0.995;
            public static double AlphaObservedMinFrac = 0.25;
            double _prevOmegaMag;
            double _alphaObserved;

            // Zero the command once the nose has arrived and is nearly still.
            public static bool TerminalLatch = true;
            public static double TerminalLatchAngleRad = 1.0 * Math.PI / 180.0;
            public static double TerminalLatchRate = 0.05;   // rad/s

            public Vector3D? OmegaTrack = null;     // planned body-frame rate (rad/s); null = off
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
            static Vector3D AccelToRateGap(Vector3D aDesired)
            {
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
                    if (measuredNow > _alphaObserved) _alphaObserved = measuredNow;
                    else _alphaObserved *= Math.Pow(AlphaObservedDecay, h * 60.0);
                }

                double dot = Vector3D.Dot(targetDir, currentFwd);
                if (dot > 1.0) dot = 1.0;
                if (dot < -1.0) dot = -1.0;
                double angleRad = Math.Acos(dot);
                double angleDeg = angleRad * 180.0 / Math.PI;
                LastAngleRad = angleRad;

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
                        if (iEffR < 1.0) iEffR = 1.0;
                        double alphaDerived = Ship.MaxTorque / iEffR;

                        double alphaEff = _alphaObserved >= alphaDerived * AlphaObservedMinFrac
                            ? _alphaObserved
                            : alphaDerived;
                        if (alphaEff > 1e-6)
                        {
                            double omBrakeable = Math.Sqrt(2.0 * alphaEff * angleRad);
                            if (omegaErr > omBrakeable) omegaErr = omBrakeable;
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
                    if (iEff < 1.0) iEff = 1.0;
                    // Prefer the MEASURED capability for the brake point.
                    double alphaDerivedBB = Ship.MaxTorque / iEff;
                    double alphaFloorBB = alphaDerivedBB * AlphaObservedMinFrac;
                    double alphaMeasBB = _alphaObserved > alphaFloorBB ? _alphaObserved : alphaFloorBB;
                    double alphaMax = Math.Min(alphaDerivedBB, alphaMeasBB);
                    if (alphaMax < 1e-3) alphaMax = 1e-3;
                    double brakingAngle = angVelMag * angVelMag / (2.0 * alphaMax);

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
                        double alphaCmd = alphaDerivedBB;
                        if (_alphaObserved > alphaCmd) alphaCmd = _alphaObserved;
                        // MUST be the number the adapter multiplies the command by, so read it from the
                        // ship rather than a field a caller may not have set.
                        double rateCapIP = Ship.GyroRateCap;
                        if (!(rateCapIP > 1e-6)) rateCapIP = CommandedRateCapBase;

                        // Work relative to the setpoint's own motion, so tracking a moving target needs
                        // no extra term.
                        Vector3D omegaRel = omegaBody - Vector3D.Transform(_setpointRateWorld, invOrient);
                        double omegaU = Vector3D.Dot(omegaRel, u);   // closing rate about u
                        double v = -omegaU;                          // d(theta)/dt

                        double alphaSwitch = alphaMax;
                        double s = angleRad + v * Math.Abs(v) / (2.0 * alphaSwitch) + v * h * 0.5;
                        // One control period's uncertainty in s. Inside that band the sign of s is not
                        // knowable from sampled state; null the residual rate instead of banging on a
                        // coin-flip. Derived band, not a tuned deadband.
                        double sBand = Math.Abs(v) * h * 0.5
                                     + 0.5 * alphaSwitch * h * h;
                        double aU = Math.Abs(s) < sBand
                            ? v / h                                 // kill the residual rate this period
                            : (s > 0.0 ? alphaCmd : -alphaCmd);     // otherwise full authority, correct sign
                        if (aU > alphaCmd) aU = alphaCmd; else if (aU < -alphaCmd) aU = -alphaCmd;
                        // Hardware rate ceiling: coast there rather than command past it.
                        if (aU > 0.0 && omegaU >= rateCapIP) aU = 0.0;

                        // Rotation off the closing axis does no useful work: deadbeat it in one tick and
                        // clamp the SUM. Giving the along-axis first claim leaves the perpendicular
                        // nothing, and the ship tumbles about an undamped axis at the rate cap.
                        Vector3D omegaPerp = omegaRel - omegaU * u;
                        Vector3D aDes = u * aU - omegaPerp * (1.0 / h);
                        double aMag = aDes.Length();
                        if (aMag > alphaCmd && aMag > 1e-9) aDes *= alphaCmd / aMag;
                        // Invert the servo: the gap that yields exactly aDes.
                        Vector3D omegaCmd = omegaBody + AccelToRateGap(aDes);
                        // Back to command space (anti-parallel, normalised by the rate cap).
                        control = -omegaCmd / rateCapIP;
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

            // Last measured nose error, for status reporting (the mod exposes this as AutoRotate_Angle).
            public double LastAngleRad { get; private set; }
        }
    }
}

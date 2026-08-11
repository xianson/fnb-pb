using System;
using VRageMath;

namespace IngameScript
{
    partial class Program
    {
        // Deliberate gyro torque measurement: drive each body axis to saturation, watch how fast the
        // ship actually accelerates, and back out torque = alpha * I about that axis.
        //
        // This exists because a PB cannot read MyGyroDefinition.ForceMagnitude, so the alternative is
        // trusting a hardcoded vanilla constant that any gyro mod invalidates. Passive measurement
        // during a mission only ever samples the authority the mission happened to demand; this asks
        // the question directly.
        //
        // SE clamps gyro output to a sphere, so authority is isotropic. All three axes are measured
        // anyway and the largest kept: each individual figure is a LOWER bound, because the override
        // servo ramps in over up to 120 frames and then coasts at the rate cap where alpha is ~0.
        public sealed class GyroCalibrator
        {
            public enum Stage { Spin, Counter, Settle, Done }

            public static string StageName(Stage s)
            {
                switch (s)
                {
                    case Stage.Spin: return "spin";
                    case Stage.Counter: return "counter";
                    case Stage.Settle: return "settle";
                    case Stage.Done: return "done";
                }
                return "?";
            }

            readonly IShip _ship;
            public Stage CurrentStage = Stage.Spin;
            public int Axis;                    // 0 = pitch, 1 = yaw, 2 = roll

            // Long enough to clear the servo ramp and reach peak alpha, short enough to stay polite.
            public double SpinSeconds = 2.5;
            public double SettleSeconds = 1.5;

            // Commanded spin rate during calibration. Kept moderate on purpose: the PB's only velocity
            // source (GetShipVelocities) under-reads very fast rotation, so a flat-out spin measures far
            // below the true authority. This stays inside the range the sensor reports accurately.
            public static double CalibrateRateCap = 1.5;

            double _t;
            double _prevOmegaAxis;
            bool _havePrev;
            readonly double[] _peakAlpha = new double[3];
            // Highest |omega| reached on each axis. The per-tick alpha derivative badly under-reads a
            // fast ramp (readback lag), so a robust floor is that the ship reached this rate within the
            // spin window: alpha >= peakOmega / SpinSeconds. Immune to per-tick noise.
            readonly double[] _peakOmegaAxis = new double[3];

            // Driving a gyro to saturation also reveals the world's max angular speed, which is the
            // other constant a PB cannot read (the mod gets it from the environment definition).
            // Only trusted once the rate has visibly PLATEAUED, otherwise it is just wherever the
            // spin happened to get to before the stage timed out.
            double _peakOmega;
            bool _plateaued;
            public double ResultRateCap { get; private set; }
            public bool RateCapValid { get; private set; }

            public double ResultTorque { get; private set; }
            public bool IsDone { get { return CurrentStage == Stage.Done; } }

            public GyroCalibrator(IShip ship)
            {
                _ship = ship;
            }

            public string Progress()
            {
                if (CurrentStage == Stage.Done) return "done";
                string ax = Axis == 0 ? "pitch" : (Axis == 1 ? "yaw" : "roll");
                return ax + " " + StageName(CurrentStage) + " " + _t.ToString("0.0", Inv) + "s";
            }

            public void Update(double dt)
            {
                if (CurrentStage == Stage.Done) return;

                IRigidBody rb = _ship.Body;
                Vector3D omegaBody = Vector3D.Transform(rb.AngularVelocity, Quaternion.Conjugate(rb.Orientation));
                double omAxis = Axis == 0 ? omegaBody.X : (Axis == 1 ? omegaBody.Y : omegaBody.Z);

                // Sample only while actually commanding authority; Settle would measure drag, not gyros.
                if (_havePrev && dt > 1e-4 && CurrentStage != Stage.Settle)
                {
                    double alpha = Math.Abs(omAxis - _prevOmegaAxis) / dt;
                    if (alpha > _peakAlpha[Axis]) _peakAlpha[Axis] = alpha;

                    double om = Math.Abs(omAxis);
                    if (om > _peakOmega) _peakOmega = om;
                    // Floor uses the WORLD-frame total rate, not the body-axis component: while spinning,
                    // the body frame rotates with the ship, so the commanded-axis component collapses as
                    // it tumbles even though the ship is turning fast. The magnitude is frame-invariant.
                    double omTotal = rb.AngularVelocity.Length();
                    if (omTotal > _peakOmegaAxis[Axis]) _peakOmegaAxis[Axis] = omTotal;
                    // Still commanding full authority but no longer accelerating: the rate cap binds.
                    if (_peakAlpha[Axis] > 1e-6 && alpha < 0.05 * _peakAlpha[Axis] && om > 0.5 * _peakOmega)
                    {
                        _plateaued = true;
                        if (om > ResultRateCap) ResultRateCap = om;
                    }
                }
                _prevOmegaAxis = omAxis;
                _havePrev = true;

                // Dampeners OFF while spinning: on an RCS-assisted gyro hull, dampener thrust is
                // excluded from producing rotation, so holding station starves the spin and the
                // measured alpha reads far below the real authority. Re-enable only in Settle to
                // arrest the residual rate/drift between axes. Thrusters stay at 0 override.
                _ship.ThrottleForward = 0.0;
                _ship.StrafeRight = 0.0;
                _ship.StrafeUp = 0.0;
                _ship.DampenersWanted = (CurrentStage == Stage.Settle);

                double cmd;
                switch (CurrentStage)
                {
                    case Stage.Spin: cmd = 1.0; break;
                    // Reverse rather than coast: it both nulls the rate it just built and yields a
                    // second, usually larger, alpha sample from a bigger commanded rate gap.
                    case Stage.Counter: cmd = -1.0; break;
                    default: cmd = 0.0; break;
                }

                _ship.Pitch = Axis == 0 ? cmd : 0.0;
                _ship.Yaw = Axis == 1 ? cmd : 0.0;
                _ship.Roll = Axis == 2 ? cmd : 0.0;

                _t += dt;
                double limit = CurrentStage == Stage.Settle ? SettleSeconds : SpinSeconds;
                if (_t < limit) return;

                _t = 0.0;
                _havePrev = false;
                switch (CurrentStage)
                {
                    case Stage.Spin:
                        CurrentStage = Stage.Counter;
                        break;
                    case Stage.Counter:
                        CurrentStage = Stage.Settle;
                        break;
                    default:
                        if (Axis >= 2) Finish();
                        else { Axis++; CurrentStage = Stage.Spin; }
                        break;
                }
            }

            void Finish()
            {
                Vector3D I = _ship.Body.InertiaBody;
                double best = 0.0;
                for (int a = 0; a < 3; a++)
                {
                    double inertia = a == 0 ? I.X : (a == 1 ? I.Y : I.Z);
                    // Take the larger of the per-tick peak and the reached-rate floor.
                    double alpha = Math.Max(_peakAlpha[a], _peakOmegaAxis[a] / Math.Max(SpinSeconds, 1e-3));
                    double tq = alpha * inertia;
                    if (tq > best) best = tq;
                }
                ResultTorque = best;
                RateCapValid = _plateaued && ResultRateCap > 1e-3;
                if (ResultRateCap > OrientationController.GyroHardwareRateCap)
                    ResultRateCap = OrientationController.GyroHardwareRateCap;
                CurrentStage = Stage.Done;
                _ship.Pitch = 0.0; _ship.Yaw = 0.0; _ship.Roll = 0.0;
            }
        }
    }
}

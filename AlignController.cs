using System;
using VRageMath;

namespace IngameScript
{
    partial class Program
    {
        // Matches the mod's RotationMode (GyroController.cs). Values kept identical so the numeric
        // form in CustomData maps to the mod's.
        public enum RotationMode
        {
            None = 0,
            Prograde = 1,
            Retrograde = 2,
            Gravity = 3,
            GPS = 4,
            Target = 6
        }

        public static string ModeName(RotationMode m)
        {
            switch (m)
            {
                case RotationMode.None: return "None";
                case RotationMode.Prograde: return "Prograde";
                case RotationMode.Retrograde: return "Retrograde";
                case RotationMode.Gravity: return "Gravity";
                case RotationMode.GPS: return "GPS";
                case RotationMode.Target: return "Target";
            }
            return "None";
        }

        // Standalone alignment. Port of GyroController.TryGetTargetDirection driving the same
        // OrientationController the autopilot uses, so alignment behaves identically either way.
        public sealed class AlignController
        {
            readonly IShip _ship;
            readonly OrientationController _att;

            public RotationMode Mode = RotationMode.None;
            public Vector3D GpsTarget;
            public bool HasGpsTarget;

            // ROLL REFERENCE. Align aims one axis; the roll about it is free, and the shortest-arc
            // solve lands on whatever the geometry gives. Set this to a world direction and the
            // ship's up axis is held as close to it as the aim allows, so repeated aligns onto the
            // same bearing come back to the SAME attitude rather than a rolled one. Only the
            // component perpendicular to the aim is used; a reference parallel to it is ignored.
            public Vector3D RollRef;
            public bool HasRollRef;
            // Roll error about the aim axis, degrees. 180 when no reference is set.
            public double RollDeg { get; private set; }

            // Below this speed the retrograde nose FREEZES on the last clean -v/|v| instead of chasing
            // a direction that becomes ill-conditioned as v -> 0.
            const double RetroHoldFreezeSpeed = 10.0;   // m/s
            Vector3D _retroAxisLatched;
            bool _retroAxisLatchValid;

            // Reported for status; mirrors the mod's AutoRotate_Aligned / AutoRotate_Angle.
            public bool IsAligned { get; private set; }
            public double AngleDeg { get; private set; }
            public bool HasDirection { get; private set; }

            // GyroController reports aligned inside 2 deg (IsAligned = AngleToTarget < 2f). This is the
            // reported flag only -- it is not the attitude law's terminal latch, which is a separate
            // 1 deg command-zeroing band.
            public double AlignedToleranceDeg = 2.0;
            // Looser than the aim tolerance on purpose: roll authority is the weakest axis on a long
            // hull (roll inertia is the small one, but the reference is only meaningful to within
            // how well the aim itself is held), and a tight band here stalls the aligned flag.
            public double RollToleranceDeg = 5.0;

            // Signed-magnitude roll error about the aim axis: how far the ship's up has rotated away
            // from the reference, measured in the plane perpendicular to where we are pointing.
            double MeasureRollDeg(Vector3D dir)
            {
                Vector3D want = RollRef - dir * Vector3D.Dot(RollRef, dir);
                if (want.LengthSquared() < 1e-9) return 0.0;      // no roll information in it
                // IShip exposes only Forward, so take up from the body orientation directly.
                Vector3D shipUp = Vector3D.Transform(Vector3D.Up, _ship.Body.Orientation);
                Vector3D have = shipUp - dir * Vector3D.Dot(shipUp, dir);
                if (have.LengthSquared() < 1e-9) return 0.0;
                return MathHelpers.AngleBetween(have, want) * 180.0 / Math.PI;
            }

            public AlignController(IShip ship, OrientationController att)
            {
                _ship = ship;
                _att = att;
                AngleDeg = 180.0;
            }

            public void Reset()
            {
                _retroAxisLatchValid = false;
                IsAligned = false;
                AngleDeg = 180.0;
                HasDirection = false;
            }

            // Returns false when the mode has no usable direction right now (e.g. Prograde at rest);
            // the caller should leave the gyros alone rather than aim at noise.
            public bool TryGetTargetDirection(out Vector3D direction)
            {
                direction = Vector3D.Zero;
                switch (Mode)
                {
                    case RotationMode.Prograde:
                        {
                            Vector3D vel = _ship.Body.LinearVelocity;
                            if (vel.LengthSquared() < 1.0) return false;
                            direction = Vector3D.Normalize(vel);
                            return true;
                        }
                    case RotationMode.Retrograde:
                        {
                            Vector3D vel = _ship.Body.LinearVelocity;
                            double sp = vel.Length();
                            if (sp > RetroHoldFreezeSpeed)
                            {
                                _retroAxisLatched = -vel / sp;
                                _retroAxisLatchValid = true;
                                direction = _retroAxisLatched;
                                return true;
                            }
                            if (_retroAxisLatchValid)
                            {
                                direction = _retroAxisLatched;
                                return true;
                            }
                            if (sp < 1.0) return false;
                            direction = -vel / sp;
                            return true;
                        }
                    case RotationMode.Gravity:
                        {
                            Vector3D grav = _ship.Gravity;
                            if (grav.LengthSquared() < 0.0001) return false;
                            // Nose points AWAY from the source (up).
                            direction = -Vector3D.Normalize(grav);
                            return true;
                        }
                    // Target has no PB equivalent (it reads the WeaponCore AI focus), so it aliases GPS.
                    case RotationMode.GPS:
                    case RotationMode.Target:
                        {
                            if (!HasGpsTarget) return false;
                            Vector3D toward = GpsTarget - _ship.Body.Position;
                            if (toward.LengthSquared() < 100.0) return false;   // within 10 m
                            direction = Vector3D.Normalize(toward);
                            return true;
                        }
                    default:
                        return false;
                }
            }

            // Drives the attitude law for one period. Returns true if gyros were commanded.
            public bool Update()
            {
                if (Mode == RotationMode.None)
                {
                    IsAligned = false;
                    HasDirection = false;
                    AngleDeg = 180.0;
                    return false;
                }

                Vector3D dir;
                if (!TryGetTargetDirection(out dir))
                {
                    HasDirection = false;
                    IsAligned = false;
                    _ship.Pitch = 0.0; _ship.Yaw = 0.0; _ship.Roll = 0.0;
                    return false;
                }

                HasDirection = true;
                _att.Target = HasRollRef
                    ? MathHelpers.LookAlong(dir, RollRef)
                    : MathHelpers.LookAlong(dir);
                _att.Update();

                AngleDeg = MathHelpers.AngleBetween(_ship.Forward, dir) * 180.0 / Math.PI;
                RollDeg = HasRollRef ? MeasureRollDeg(dir) : 180.0;
                // With a reference set, "aligned" has to mean the roll landed too -- the caller is
                // holding out for a repeatable attitude, not just a bearing.
                IsAligned = AngleDeg <= AlignedToleranceDeg
                         && (!HasRollRef || RollDeg <= RollToleranceDeg);
                return true;
            }

            public static bool TryParse(string s, out RotationMode mode)
            {
                mode = RotationMode.None;
                if (string.IsNullOrEmpty(s)) return false;
                switch (s.Trim().ToLowerInvariant())
                {
                    case "none": case "off": case "0": mode = RotationMode.None; return true;
                    case "prograde": case "pro": case "1": mode = RotationMode.Prograde; return true;
                    case "retrograde": case "retro": case "2": mode = RotationMode.Retrograde; return true;
                    case "gravity": case "grav": case "3": mode = RotationMode.Gravity; return true;
                    case "gps": case "4": mode = RotationMode.GPS; return true;
                    case "target": case "tgt": case "6": mode = RotationMode.Target; return true;
                }
                return false;
            }
        }
    }
}

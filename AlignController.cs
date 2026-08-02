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
                _att.Target = MathHelpers.LookAlong(dir);
                _att.Update();

                AngleDeg = MathHelpers.AngleBetween(_ship.Forward, dir) * 180.0 / Math.PI;
                IsAligned = AngleDeg <= AlignedToleranceDeg;
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

using System;
using VRageMath;

namespace IngameScript
{
    partial class Program
    {
        // Verbatim port of the mod's WaypointQueue/MathHelpers.cs.
        public static class MathHelpers
        {
            // Quaternion that rotates the ship's nose (-UnitZ) to point along world-space `dir`.
            public static Quaternion LookAlong(Vector3D dir)
            {
                if (dir.LengthSquared() < 1e-12) return Quaternion.Identity;
                dir = Vector3D.Normalize(dir);
                Vector3D nose = -Vector3D.UnitZ;
                double dot = Vector3D.Dot(nose, dir);
                if (dot > 0.9999999) return Quaternion.Identity;
                if (dot < -0.9999999) return Quaternion.CreateFromAxisAngle(Vector3.UnitY, (float)Math.PI);
                Vector3D axis = Vector3D.Normalize(Vector3D.Cross(nose, dir));
                float angle = (float)Math.Acos(dot);
                Vector3 axisF = new Vector3((float)axis.X, (float)axis.Y, (float)axis.Z);
                return Quaternion.CreateFromAxisAngle(axisF, angle);
            }

            // Unsigned angle between two world-frame vectors, radians.
            public static double AngleBetween(Vector3D a, Vector3D b)
            {
                double la = a.Length();
                double lb = b.Length();
                if (la < 1e-9 || lb < 1e-9) return 0.0;
                double c = Vector3D.Dot(a, b) / (la * lb);
                if (c > 1.0) c = 1.0;
                if (c < -1.0) c = -1.0;
                return Math.Acos(c);
            }

            public static double Clamp1(double v)
            {
                if (v < -1.0) return -1.0;
                if (v > 1.0) return 1.0;
                return v;
            }

            // Per-sign strafe command normalization.
            public static double StrafeCmd(double aReq, double aPos, double aNeg)
            {
                double budget = aReq >= 0.0 ? aPos : aNeg;
                if (budget <= 1e-9) return 0.0;
                return Clamp1(aReq / budget);
            }

            // Gravity-compensated thrust command.
            public static void GravityCompensatedCommand(
                Vector3D aDesired, Vector3D gravity, double aMax,
                Vector3D fallbackDir,
                out Vector3D noseDir, out double throttle)
            {
                Vector3D aThrust = aDesired - gravity;
                double mag = aThrust.Length();
                if (mag < 1e-6)
                {
                    // No demand and no gravity: keep the phase's preferred nose so the ship does not wander.
                    double fbLen = fallbackDir.Length();
                    noseDir = fbLen > 1e-6 ? fallbackDir / fbLen : -Vector3D.UnitZ;
                    throttle = 0.0;
                    return;
                }
                noseDir = aThrust / mag;
                throttle = aMax > 1e-6 ? Math.Min(1.0, mag / aMax) : 0.0;
            }
        }
    }
}

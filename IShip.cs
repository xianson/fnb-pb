using VRageMath;

namespace IngameScript
{
    partial class Program
    {
        // Abstract ship surface the controllers are written against. Ported from the mod's
        // WaypointQueue/IShip.cs so the control laws come across unchanged.
        public interface IShip
        {
            IRigidBody Body { get; }

            // World-frame nose direction (= refMatrix.Forward = world dir of body -Z).
            Vector3D Forward { get; }

            // World-frame gravity at centre of mass (m/s^2), toward the source; zero in space.
            Vector3D Gravity { get; }

            // Gravity at an arbitrary world point. A PB cannot sample this, so the adapter returns
            // the ship's own gravity; only the (unported) schedule bake ever wanted a real answer.
            Vector3D GravityAt(Vector3D worldPos);

            // Per-axis thrust limits in body frame (Newtons); aMax = MaxXThrust * InvMass.
            double MaxForwardThrust { get; }
            double MaxBackwardThrust { get; }
            double MaxRightThrust { get; }
            double MaxLeftThrust { get; }
            double MaxUpThrust { get; }
            double MaxDownThrust { get; }
            double MaxLateralThrust { get; }

            // Total gyro authority (N*m).
            double MaxTorque { get; }

            // Gyro max sustained angular-rate cap (rad/s). Also the scale ApplyGyros multiplies
            // the [-1,1] command by, so the attitude law must normalise by the same number.
            double GyroRateCap { get; }

            // Per-tick commands, [-1,1] as a fraction of the budget.
            double ThrottleForward { get; set; }
            double StrafeRight { get; set; }
            double StrafeUp { get; set; }

            // Body-frame torque commands in [-1,1].
            double Pitch { get; set; }
            double Yaw { get; set; }
            double Roll { get; set; }

            // Adapter enables DampenersOverride on every ship controller so SE's own damping finishes the stop.
            bool DampenersWanted { get; set; }

            bool SportMode { get; }

            // Atmospheric drag decel on the cruise velocity, m/s^2. Not observable from a PB; always 0.
            double BrakeDragDecel { get; }

            // Forces an immediate mass/inertia/thrust/torque re-read. The budget is otherwise refreshed
            // lazily (~0.5 s cadence), so a check that runs on the first tick after engage -- the flip
            // gate -- would otherwise judge a stale mass (e.g. a just-emptied hold still reads laden).
            void ForceBudgetRefresh();
        }

        public interface IRigidBody
        {
            Vector3D Position { get; }
            // Controller-relevant velocity. Under the mod's HighSpeed cruise this is the VIRTUAL
            // velocity (thousands of m/s), derived here by differencing world position.
            Vector3D LinearVelocity { get; }
            // SE physics velocity, clamped (~95 m/s) while HighSpeed is engaged.
            Vector3D PhysicalLinearVelocity { get; }
            Vector3D AngularVelocity { get; }   // world frame, rad/s
            Quaternion Orientation { get; }     // body -> world
            Vector3D InertiaBody { get; }       // principal-axis moments
            double Mass { get; }
            double InvMass { get; }
        }
    }
}

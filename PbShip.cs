using System;
using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using VRage.Game.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    partial class Program
    {
        // IShip backed by terminal blocks. The PB analogue of the mod's WaypointQueue/SEShip.cs.
        public sealed class PbShip : IShip
        {
            // Bucket order for the cached thruster->axis assignment.
            const int BFwd = 0, BBack = 1, BRight = 2, BLeft = 3, BUp = 4, BDown = 5;

            readonly IMyGridTerminalSystem _gts;
            readonly IMyProgrammableBlock _me;
            readonly Action<string> _log;

            IMyShipController _ctrl;
            public string ControllerNameHint = "";

            readonly List<IMyThrust> _thrusters = new List<IMyThrust>();
            readonly List<int> _bucket = new List<int>();          // parallel to _thrusters
            readonly List<IMyGyro> _gyros = new List<IMyGyro>();

            // WRITE-ON-CHANGE. A terminal property write is a sync-value set plus its change
            // notification -- cheap individually, but this ship writes one per thruster and four per
            // gyro EVERY run, and that is wall-clock the instruction counter never sees. The blocks
            // retain the last value written, so re-writing an identical value is a no-op and skipping
            // it cannot change flight. Comparison is EXACT: an epsilon would round the command, which
            // would be a behaviour change.
            readonly List<float> _lastPct = new List<float>();     // parallel to _thrusters
            readonly List<float> _lastGx = new List<float>();      // parallel to _gyros
            readonly List<float> _lastGy = new List<float>();
            readonly List<float> _lastGz = new List<float>();
            readonly List<bool> _lastGOn = new List<bool>();
            // Liveness flags, sampled with the thrust/torque budgets rather than per run. Reading
            // IsWorking/IsFunctional/Enabled on every block on every run was ~130 ModAPI property reads
            // per run on this hull, for a set that the budgets themselves only re-read twice a second.
            readonly List<bool> _thrOk = new List<bool>();
            readonly List<bool> _gyroOk = new List<bool>();
            // Another script or the pilot can move an override behind our back, which would strand the
            // cache. One block per run is re-written unconditionally, so any desync is corrected
            // within this many runs at no measurable cost.
            public int ForceWriteCycle = 60;
            int _writePhase;
            readonly List<IMyShipController> _controllers = new List<IMyShipController>();
            // Previous scan's actuators, kept only long enough to release any that left the construct.
            readonly List<IMyThrust> _prevThrusters = new List<IMyThrust>();
            readonly List<IMyGyro> _prevGyros = new List<IMyGyro>();
            readonly List<IMyUpgradeModule> _upgrades = new List<IMyUpgradeModule>();

            MatrixD _refMatrix;
            Vector3D _position, _linearVelocity, _physicalVelocity, _angularVelocity;
            Quaternion _orientation = Quaternion.Identity;
            Vector3D _forward = -Vector3D.UnitZ;
            Vector3D _gravity;
            Vector3D _inertiaBody = Vector3D.One;
            double _mass = 1.0, _invMass = 1.0;

            double _maxForwardThrust, _maxBackwardThrust;
            double _maxRightThrust, _maxLeftThrust;
            double _maxUpThrust, _maxDownThrust;
            double _maxLateralThrust;
            double _maxTorque;

            double _throttleForward, _strafeRight, _strafeUp;
            double _pitch, _yaw, _roll;
            bool _dampenersWanted;
            bool _dampenersValid;
            bool _dampenersApplied;
            bool _hasOverrides;

            Vector3D _driveDirLocal = -Vector3D.UnitZ;
            Vector3D _driveUpLocal = Vector3D.UnitY;
            bool _driveAxisResolved;

            int _runsSinceScan = int.MaxValue;
            int _runsSinceBudget = int.MaxValue;
            // Block re-scan cadence, in PB runs. Cheap enough at ~5 s but far too expensive per tick.
            public int ScanIntervalRuns = 300;
            // Thrust/torque budget cadence. MaxEffectiveThrust moves with altitude, so this cannot be
            // scan-rate, but it does not need to be tick-rate either.
            public int BudgetIntervalRuns = 30;

            // ---- Tunables the PB cannot read from the world (see README) ----
            public double GyroRateCapOverride = OrientationController.CommandedRateCapBase;

            // GYRO TORQUE. The mod reads MyGyroDefinition.ForceMagnitude; that type is not on the PB
            // whitelist, and hardcoding the vanilla numbers is wrong the moment anyone runs a modded
            // gyro. So it is MEASURED: torque = |domega/dt| * I about the observed rotation axis,
            // peak-held with decay, sampled only while we are commanding near-saturation authority.
            // The definition constants below are a COLD-START SEED only, used until the first slew
            // produces a real measurement -- something is needed for the very first flip estimate.
            // Set GyroTorqueOverride (CustomData: GyroTorque) to pin it explicitly.
            public double GyroTorqueOverride;          // 0 = auto (measure)
            public double SeedLargeGyroTorque = 3.36e7;  // N*m, vanilla LargeBlockGyro ForceMagnitude
            public double SeedSmallGyroTorque = 4.48e5;  // N*m, vanilla SmallBlockGyro ForceMagnitude

            double _torqueSeed;
            double _torqueObserved;
            bool _torqueCalibrated;
            Vector3D _prevOmegaBody;
            bool _havePrevOmega;
            // Per-run decay on the observed peak. Slow: a full flip only shows peak authority briefly.
            public double TorqueObservedDecay = 0.999;
            // Only believe a sample when we are actually asking for most of the available authority.
            public double TorqueSampleCmdMin = 0.5;

            // ---- Velocity source ----
            // The mod publishes FB_Velocity on the Flight Computer: the SAME IRigidBody.LinearVelocity
            // its own controllers plan against (virtual in HighSpeed, SE physics otherwise). When that
            // is available it is strictly better than differencing position -- no filter lag, no
            // spike gate, no CoM-jump artefacts -- so it wins. Position-differencing stays as the
            // fallback for a world without the mod, or a ship with no flight computer.
            IMyTerminalBlock _fcBlock;
            ITerminalProperty<Vector3D> _fcVel, _fcPhysVel;
            bool _modVelocityOk;
            public bool UsingModVelocity { get { return _modVelocityOk; } }

            // ---- Derived-velocity filter (fallback) ----
            Vector3D _prevPos;
            bool _havePrevPos, _haveVel;
            int _velSamples;
            int _rejectRun;
            public double VelSmooth = 0.5;             // EMA on the position-differenced velocity
            public double SpikeAccelMult = 4.0;        // plausible dv = accel*dt*mult + floor
            public double SpikeFloorMps = 50.0;
            public int MaxConsecutiveRejects = 3;      // then accept, so a real discontinuity recovers
            public double MinDtSeconds = 0.008;

            public PbShip(IMyGridTerminalSystem gts, IMyProgrammableBlock me, Action<string> log)
            {
                _gts = gts;
                _me = me;
                _log = log;
            }

            public IMyShipController Controller { get { return _ctrl; } }

            // False until RefreshState has succeeded once. Position and orientation are meaningless
            // before that -- Position reads Vector3D.Zero -- so anything measured relative to the
            // ship has to check this first.
            public bool StateValid { get { return _stateValid; } }
            bool _stateValid;

            // Drops the position history and the sample count so DerivedVelocityValid genuinely
            // waits for fresh readings. RefreshState only runs while a mission is live, so without
            // this the estimator carries the previous flight's velocity across the idle gap and
            // still reports itself valid.
            public void ResetVelocityEstimator()
            {
                _havePrevPos = false;
                _haveVel = false;
                _velSamples = 0;
                _rejectRun = 0;
            }
            // The mod's reading is good immediately. The fallback needs two accepted samples: the
            // first is a bare difference, the second has been through the EMA, and the flip/brake
            // plan is solved once from this number.
            public bool DerivedVelocityValid { get { return _modVelocityOk || _velSamples >= 2; } }

            // True when the NEXT RefreshState will do a full block rescan, so the host can label the
            // run. The rescan is four GridTerminalSystem queries plus a terminal-property lookup and is
            // by far the most expensive thing this class does.
            public bool WillRescan
            {
                get { return _runsSinceScan >= ScanIntervalRuns || _ctrl == null || !IsAlive(_ctrl); }
            }

            // ---------- IShip ----------

            public IRigidBody Body { get { return _bodyView ?? (_bodyView = new BodyView(this)); } }
            BodyView _bodyView;

            public Vector3D Forward { get { return _forward; } }
            public Vector3D Gravity { get { return _gravity; } }
            // A PB cannot sample gravity away from the ship; only the unported schedule bake wanted it.
            public Vector3D GravityAt(Vector3D worldPos) { return _gravity; }

            public double MaxForwardThrust { get { return _maxForwardThrust; } }
            public double MaxBackwardThrust { get { return _maxBackwardThrust; } }
            public double MaxRightThrust { get { return _maxRightThrust; } }
            public double MaxLeftThrust { get { return _maxLeftThrust; } }
            public double MaxUpThrust { get { return _maxUpThrust; } }
            public double MaxDownThrust { get { return _maxDownThrust; } }
            public double MaxLateralThrust { get { return _maxLateralThrust; } }
            public double MaxTorque { get { return _maxTorque; } }

            public double GyroRateCap
            {
                get
                {
                    double cap = GyroRateCapOverride;
                    if (!(cap > 1e-6)) cap = OrientationController.CommandedRateCapBase;
                    if (cap > OrientationController.GyroHardwareRateCap)
                        cap = OrientationController.GyroHardwareRateCap;
                    return cap;
                }
            }

            public double ThrottleForward { get { return _throttleForward; } set { _throttleForward = Clamp1(value); } }
            public double StrafeRight { get { return _strafeRight; } set { _strafeRight = Clamp1(value); } }
            public double StrafeUp { get { return _strafeUp; } set { _strafeUp = Clamp1(value); } }
            public double Pitch { get { return _pitch; } set { _pitch = Clamp1(value); } }
            public double Yaw { get { return _yaw; } set { _yaw = Clamp1(value); } }
            public double Roll { get { return _roll; } set { _roll = Clamp1(value); } }
            public bool DampenersWanted { get { return _dampenersWanted; } set { _dampenersWanted = value; } }

            // Sport mode is a mod-side synced flag with no PB equivalent.
            public bool SportMode { get { return false; } }
            // Drag decel on the virtual cruise velocity is not observable from a PB.
            public double BrakeDragDecel { get { return 0.0; } }

            sealed class BodyView : IRigidBody
            {
                readonly PbShip _s;
                public BodyView(PbShip s) { _s = s; }
                public Vector3D Position { get { return _s._position; } }
                public Vector3D LinearVelocity { get { return _s._linearVelocity; } }
                public Vector3D PhysicalLinearVelocity { get { return _s._physicalVelocity; } }
                public Vector3D AngularVelocity { get { return _s._angularVelocity; } }
                public Quaternion Orientation { get { return _s._orientation; } }
                public Vector3D InertiaBody { get { return _s._inertiaBody; } }
                public double Mass { get { return _s._mass; } }
                public double InvMass { get { return _s._invMass; } }
            }

            // ---------- Per-run state refresh ----------

            public bool RefreshState(double dt)
            {
                if (_runsSinceScan++ >= ScanIntervalRuns || _ctrl == null || !IsAlive(_ctrl))
                {
                    _runsSinceScan = 0;
                    RescanBlocks();
                }
                if (_ctrl == null) return false;

                _position = _ctrl.CenterOfMass;

                MyShipVelocities vel = _ctrl.GetShipVelocities();
                _physicalVelocity = vel.LinearVelocity;
                _angularVelocity = vel.AngularVelocity;
                bool haveModVel = TryReadModVelocity();
                _gravity = _ctrl.GetNaturalGravity();

                // Mass, inertia and the thrust/torque budgets are the expensive reads (they walk every
                // thruster). Same 0.5 s cadence the mod's SEShip uses, rather than every run.
                if (_runsSinceBudget++ >= BudgetIntervalRuns)
                {
                    _runsSinceBudget = 0;
                    _mass = _ctrl.CalculateShipMass().PhysicalMass;
                    if (!(_mass > 1e-6)) _mass = 1.0;
                    _invMass = 1.0 / _mass;
                    RefreshInertia();
                    RefreshBudgets();
                }

                // Keep the position history current either way, so losing the flight computer
                // mid-flight falls back to a warm estimator rather than a cold one.
                UpdateDerivedVelocity(_position, dt, haveModVel);

                // Rotate the grid matrix so Forward is the main-drive direction; ThrottleForward then
                // means "push along the drive axis" whatever the grid's nominal forward is.
                MatrixD gridMatrix = _me.CubeGrid.WorldMatrix;
                Vector3D fwdWorld = Vector3D.TransformNormal(_driveDirLocal, gridMatrix);
                Vector3D upWorld = Vector3D.TransformNormal(_driveUpLocal, gridMatrix);
                // Translate at CoM so gyro (rotation-about-CoM) transforms share the origin.
                _refMatrix = MatrixD.CreateWorld(_position, fwdWorld, upWorld);
                _forward = _refMatrix.Forward;

                MatrixD rotOnly = _refMatrix;
                rotOnly.Translation = Vector3D.Zero;
                _orientation = Quaternion.CreateFromRotationMatrix(rotOnly);

                // Needs _orientation and _inertiaBody, so it runs last.
                UpdateTorqueEstimate(dt);
                _stateValid = true;
                return true;
            }

            static bool IsAlive(IMyTerminalBlock b)
            {
                return b != null && b.CubeGrid != null && !b.Closed;
            }

            // True velocity by differencing world position. Under the mod's HighSpeed cruise SE's own
            // LinearVelocity is clamped near 100 m/s while the ship really moves thousands of m/s, so
            // GetShipVelocities() would understate the brake distance by two orders of magnitude.
            void UpdateDerivedVelocity(Vector3D pos, double dt, bool modAuthoritative)
            {
                if (!_havePrevPos)
                {
                    _prevPos = pos;
                    _havePrevPos = true;
                    if (!modAuthoritative) _linearVelocity = _physicalVelocity;
                    return;
                }
                if (dt < MinDtSeconds) return;   // argument-triggered run between ticks: keep the last estimate

                Vector3D raw = (pos - _prevPos) / dt;
                _prevPos = pos;

                if (!_haveVel)
                {
                    if (!modAuthoritative) _linearVelocity = raw;
                    _haveVel = true;
                    _velSamples = 1;
                    return;
                }

                // Spike gate. CoM can jump when mass moves or a grid docks, and HighSpeed advances the
                // grid by teleport rather than smooth integration.
                double aMax = _maxForwardThrust * _invMass;
                double bound = (aMax + _gravity.Length()) * dt * SpikeAccelMult + SpikeFloorMps;
                if ((raw - _linearVelocity).Length() > bound && _rejectRun < MaxConsecutiveRejects)
                {
                    _rejectRun++;
                    return;
                }
                _rejectRun = 0;
                if (!modAuthoritative) _linearVelocity += (raw - _linearVelocity) * VelSmooth;
                if (_velSamples < 2) _velSamples = 2;
            }

            void RefreshInertia()
            {
                // Box-approx principal inertia in GRID axes, then remapped into the drive-aligned body
                // frame the controllers expect.
                IMyCubeGrid grid = _me.CubeGrid;
                Vector3I cells = grid.Max - grid.Min + Vector3I.One;
                double sx = cells.X * grid.GridSize;
                double sy = cells.Y * grid.GridSize;
                double sz = cells.Z * grid.GridSize;
                double mOver12 = _mass / 12.0;
                Vector3D inertiaGrid = new Vector3D(
                    mOver12 * (sy * sy + sz * sz),
                    mOver12 * (sx * sx + sz * sz),
                    mOver12 * (sx * sx + sy * sy));

                Vector3D refFwdLocal = _driveDirLocal;
                Vector3D refRightLocal = Vector3D.Cross(refFwdLocal, _driveUpLocal);
                if (refRightLocal.LengthSquared() > 1e-12) refRightLocal = Vector3D.Normalize(refRightLocal);
                Vector3D refUpLocal = Vector3D.Cross(refRightLocal, refFwdLocal);
                _inertiaBody = new Vector3D(
                    InertiaAboutAxis(refRightLocal, inertiaGrid),   // pitch
                    InertiaAboutAxis(refUpLocal, inertiaGrid),      // yaw
                    InertiaAboutAxis(refFwdLocal, inertiaGrid));    // roll
            }

            void RescanBlocks()
            {
                _controllers.Clear();
                _gts.GetBlocksOfType<IMyShipController>(_controllers, b => b.IsSameConstructAs(_me));
                _ctrl = PickController();

                FindFlightComputer();

                // A block that leaves the construct -- a shot-off pod, a detached rotor head -- drops
                // out of the query but keeps whatever override it was last written. The interface
                // reference stays valid after it departs, so hold the old lists and release anything
                // that is no longer ours; otherwise the detached piece burns at full thrust forever.
                _prevThrusters.Clear();
                for (int i = 0; i < _thrusters.Count; i++) _prevThrusters.Add(_thrusters[i]);
                _prevGyros.Clear();
                for (int i = 0; i < _gyros.Count; i++) _prevGyros.Add(_gyros[i]);

                _thrusters.Clear();
                _gts.GetBlocksOfType<IMyThrust>(_thrusters, b => b.IsSameConstructAs(_me));
                _gyros.Clear();
                _gts.GetBlocksOfType<IMyGyro>(_gyros, b => b.IsSameConstructAs(_me));

                ReleaseDeparted();

                // Write caches are indexed by list position, so they are rebuilt with the lists.
                _lastGx.Clear(); _lastGy.Clear(); _lastGz.Clear(); _lastGOn.Clear(); _gyroOk.Clear();
                for (int i = 0; i < _gyros.Count; i++)
                {
                    _lastGx.Add(float.NaN); _lastGy.Add(float.NaN);
                    _lastGz.Add(float.NaN); _lastGOn.Add(false); _gyroOk.Add(false);
                }

                if (_ctrl == null) return;

                // Resolve the drive axis before bucketing: the axis with the most thrust becomes
                // "forward". Every hull whose main drive is not along grid -Z needs this, and getting
                // it wrong reads MaxForwardThrust as 0 and rejects the mission outright.
                // ONCE, as SEShip does. This used to run on every rescan, so a main drive that merely
                // browned out -- H2 running thin, combat damage -- could hand "forward" to a lift bank
                // at the next scan, inverting the reference frame mid-brake.
                if (!_driveAxisResolved) ResolveDriveAxis();
                RebuildBuckets();
                // Buckets changed, so the cached budgets are stale regardless of their own cadence.
                _runsSinceBudget = int.MaxValue;
            }

            // Releases overrides on blocks that were ours last scan and are not ours now.
            void ReleaseDeparted()
            {
                for (int i = 0; i < _prevThrusters.Count; i++)
                {
                    IMyThrust t = _prevThrusters[i];
                    if (t == null || t.Closed || _thrusters.Contains(t)) continue;
                    t.ThrustOverride = 0f;
                }
                for (int i = 0; i < _prevGyros.Count; i++)
                {
                    IMyGyro g = _prevGyros[i];
                    if (g == null || g.Closed || _gyros.Contains(g)) continue;
                    // Zero the rates BEFORE dropping the override, or the setters discard them.
                    g.Pitch = 0f; g.Yaw = 0f; g.Roll = 0f;
                    g.GyroOverride = false;
                }
            }

            // The mod hangs FB_Velocity off the Flight Computer (an upgrade module). Identify it by
            // the property rather than by subtype name, so a renamed or re-subtyped block still works.
            void FindFlightComputer()
            {
                _fcBlock = null;
                _fcVel = null;
                _fcPhysVel = null;
                _modVelocityOk = false;
                _upgrades.Clear();
                _gts.GetBlocksOfType<IMyUpgradeModule>(_upgrades, b => b.IsSameConstructAs(_me));
                for (int i = 0; i < _upgrades.Count; i++)
                {
                    IMyUpgradeModule u = _upgrades[i];
                    if (u == null || u.Closed) continue;
                    try
                    {
                        ITerminalProperty p = u.GetProperty("FB_Velocity");
                        if (p == null) continue;
                        // RESOLVE THE HANDLES HERE, ONCE. GetValue<T>(string) re-enumerates the block's
                        // whole terminal-property list and allocates a List on every call; doing that
                        // twice per run was the most expensive thing on this path whenever a flight
                        // computer was present. The handles stay valid for the block's lifetime.
                        _fcVel = p.As<Vector3D>();
                        _fcPhysVel = u.GetProperty("FB_PhysicalVelocity").As<Vector3D>();
                        if (_fcVel == null || _fcPhysVel == null) { _fcVel = null; _fcPhysVel = null; continue; }
                        _fcBlock = u;
                        return;
                    }
                    catch { _fcVel = null; _fcPhysVel = null; }
                }
            }

            // Reads the mod's effective velocity. Returns false when the mod or the flight computer
            // is absent, which is the signal to fall back to position-differencing.
            bool TryReadModVelocity()
            {
                if (_fcBlock == null || _fcBlock.Closed || _fcVel == null || _fcPhysVel == null)
                {
                    _modVelocityOk = false;
                    return false;
                }
                try
                {
                    _linearVelocity = _fcVel.GetValue(_fcBlock);
                    _physicalVelocity = _fcPhysVel.GetValue(_fcBlock);
                    _modVelocityOk = true;
                    return true;
                }
                catch
                {
                    // Wrong type or the property vanished: stop asking and use the fallback.
                    _fcBlock = null;
                    _fcVel = null;
                    _fcPhysVel = null;
                    _modVelocityOk = false;
                    return false;
                }
            }

            IMyShipController PickController()
            {
                IMyShipController best = null;
                for (int i = 0; i < _controllers.Count; i++)
                {
                    IMyShipController c = _controllers[i];
                    if (c == null || c.Closed) continue;
                    if (ControllerNameHint.Length > 0)
                    {
                        if (c.CustomName.IndexOf(ControllerNameHint, StringComparison.OrdinalIgnoreCase) >= 0)
                            return c;
                        continue;
                    }
                    // No hint: prefer a working main cockpit / remote, else any working controller.
                    if (!c.IsFunctional) continue;
                    if (best == null) best = c;
                    if (c.IsMainCockpit) return c;
                }
                if (best == null && ControllerNameHint.Length > 0)
                {
                    // Named hint missed: fall back rather than refusing to fly.
                    for (int i = 0; i < _controllers.Count; i++)
                        if (_controllers[i] != null && _controllers[i].IsFunctional) return _controllers[i];
                }
                return best;
            }

            // Bucket every thruster by grid-local push axis; the largest total is the drive axis.
            void ResolveDriveAxis()
            {
                double fwd = 0, back = 0, left = 0, right = 0, up = 0, down = 0;
                MatrixD worldToGrid = MatrixD.Transpose(_me.CubeGrid.WorldMatrix);
                for (int i = 0; i < _thrusters.Count; i++)
                {
                    IMyThrust t = _thrusters[i];
                    if (t == null || t.Closed || !t.IsWorking) continue;
                    Vector3D pushGrid = Vector3D.TransformNormal(t.WorldMatrix.Backward, worldToGrid);
                    double maxT = t.MaxThrust;
                    double ax = Math.Abs(pushGrid.X), ay = Math.Abs(pushGrid.Y), az = Math.Abs(pushGrid.Z);
                    if (ax >= ay && ax >= az) { if (pushGrid.X > 0) right += maxT; else left += maxT; }
                    else if (ay >= az) { if (pushGrid.Y > 0) up += maxT; else down += maxT; }
                    else { if (pushGrid.Z < 0) fwd += maxT; else back += maxT; }
                }

                double bestThrust = -1.0;
                Vector3D driveDir = -Vector3D.UnitZ;
                if (fwd > bestThrust) { bestThrust = fwd; driveDir = -Vector3D.UnitZ; }
                if (back > bestThrust) { bestThrust = back; driveDir = Vector3D.UnitZ; }
                if (left > bestThrust) { bestThrust = left; driveDir = -Vector3D.UnitX; }
                if (right > bestThrust) { bestThrust = right; driveDir = Vector3D.UnitX; }
                if (up > bestThrust) { bestThrust = up; driveDir = Vector3D.UnitY; }
                if (down > bestThrust) { bestThrust = down; driveDir = -Vector3D.UnitY; }

                // Latch only on a real answer: a grid asked before it has power finds nothing and would
                // otherwise keep a default that is 90 deg wrong forever.
                if (!(bestThrust > 0.0) && _driveAxisResolved) return;

                _driveDirLocal = driveDir;
                Vector3D candidate = Vector3D.UnitY;
                if (Math.Abs(Vector3D.Dot(candidate, _driveDirLocal)) > 0.99)
                    candidate = Vector3D.UnitZ;
                _driveUpLocal = Vector3D.Normalize(candidate - Vector3D.Dot(candidate, _driveDirLocal) * _driveDirLocal);
                _driveAxisResolved = bestThrust > 0.0;
            }

            // Thruster orientation relative to the grid is fixed, so the axis assignment is cached and
            // the per-tick write becomes a table lookup. This is what keeps the control loop affordable.
            void RebuildBuckets()
            {
                _bucket.Clear();
                _lastPct.Clear();
                _thrOk.Clear();
                MatrixD gridMatrix = _me.CubeGrid.WorldMatrix;
                Vector3D fwdWorld = Vector3D.TransformNormal(_driveDirLocal, gridMatrix);
                Vector3D upWorld = Vector3D.TransformNormal(_driveUpLocal, gridMatrix);
                MatrixD refM = MatrixD.CreateWorld(_ctrl.CenterOfMass, fwdWorld, upWorld);
                MatrixD refToBody = MatrixD.Transpose(refM);

                for (int i = 0; i < _thrusters.Count; i++)
                {
                    IMyThrust t = _thrusters[i];
                    if (t == null || t.Closed) { _bucket.Add(BFwd); _lastPct.Add(float.NaN); _thrOk.Add(false); continue; }
                    Vector3D pushBody = Vector3D.TransformNormal(t.WorldMatrix.Backward, refToBody);
                    double ax = Math.Abs(pushBody.X), ay = Math.Abs(pushBody.Y), az = Math.Abs(pushBody.Z);
                    int b;
                    if (ax >= ay && ax >= az) b = pushBody.X > 0 ? BRight : BLeft;
                    else if (ay >= az) b = pushBody.Y > 0 ? BUp : BDown;
                    // pushBody.Z < 0 means the thruster pushes the ship forward (nozzle points +Z).
                    else b = pushBody.Z < 0 ? BFwd : BBack;
                    _bucket.Add(b);
                    _lastPct.Add(float.NaN);
                    _thrOk.Add(false);
                }
            }

            void RefreshBudgets()
            {
                double fwd = 0, back = 0, right = 0, left = 0, up = 0, down = 0;
                for (int i = 0; i < _thrusters.Count && i < _bucket.Count && i < _thrOk.Count; i++)
                {
                    IMyThrust t = _thrusters[i];
                    bool ok = t != null && !t.Closed && t.IsWorking;
                    _thrOk[i] = ok;
                    if (!ok) continue;
                    // Effective, not nominal: atmospheric/ion falloff is real brake authority lost.
                    double f = t.MaxEffectiveThrust;
                    switch (_bucket[i])
                    {
                        case BFwd: fwd += f; break;
                        case BBack: back += f; break;
                        case BRight: right += f; break;
                        case BLeft: left += f; break;
                        case BUp: up += f; break;
                        default: down += f; break;
                    }
                }

                // Same fwd/back ordering as the mod: the larger of the two is the "forward" budget, so
                // a drive axis resolved backwards still reports honest numbers.
                _maxForwardThrust = Math.Max(fwd, back);
                _maxBackwardThrust = Math.Min(fwd, back);
                _maxRightThrust = right;
                _maxLeftThrust = left;
                _maxUpThrust = up;
                _maxDownThrust = down;
                // Conservative single-scalar lateral budget (over-tilt is worse than under-tilt).
                _maxLateralThrust = Math.Min(left, Math.Min(right, Math.Min(up, down)));

                // Cold-start seed, mirroring the mod's sum of ForceMagnitude * GyroPower.
                bool small = _me.CubeGrid.GridSizeEnum == VRage.Game.MyCubeSize.Small;
                double per = small ? SeedSmallGyroTorque : SeedLargeGyroTorque;
                double seed = 0.0;
                for (int i = 0; i < _gyros.Count && i < _gyroOk.Count; i++)
                {
                    IMyGyro g = _gyros[i];
                    bool ok = g != null && !g.Closed && g.IsFunctional && g.Enabled;
                    _gyroOk[i] = ok;
                    if (!ok) continue;
                    seed += per * g.GyroPower;
                }
                _torqueSeed = seed;
            }

            // Back the real torque out of the ship's own response. Runs every tick; the peak-hold is
            // what captures authority, since a slew spends most of its time rate-capped at alpha ~ 0.
            void UpdateTorqueEstimate(double dt)
            {
                Vector3D omegaBody = Vector3D.Transform(_angularVelocity, Quaternion.Conjugate(_orientation));
                if (_havePrevOmega && dt >= MinDtSeconds)
                {
                    // Only while we are commanding most of the authority, or we would be measuring
                    // collisions and pilot input rather than the gyros.
                    double cmd = Math.Max(Math.Abs(_pitch), Math.Max(Math.Abs(_yaw), Math.Abs(_roll)));
                    if (_hasOverrides && cmd >= TorqueSampleCmdMin)
                    {
                        Vector3D dW = (omegaBody - _prevOmegaBody) / dt;
                        double mag = dW.Length();
                        if (mag > 1e-4)
                        {
                            Vector3D ax = dW / mag;
                            double iEff = ax.X * ax.X * _inertiaBody.X
                                        + ax.Y * ax.Y * _inertiaBody.Y
                                        + ax.Z * ax.Z * _inertiaBody.Z;
                            double tq = mag * iEff;
                            if (tq > _torqueObserved) { _torqueObserved = tq; _torqueCalibrated = true; }
                            else _torqueObserved *= TorqueObservedDecay;
                        }
                    }
                }
                _prevOmegaBody = omegaBody;
                _havePrevOmega = true;

                _maxTorque = GyroTorqueOverride > 0.0
                    ? GyroTorqueOverride
                    : (_torqueCalibrated ? _torqueObserved : _torqueSeed);
            }

            public bool TorqueCalibrated { get { return _torqueCalibrated; } }
            public double TorqueSeed { get { return _torqueSeed; } }

            // ---------- Per-run command application ----------

            public void ApplyCommands()
            {
                _writeCountAccum = 0;
                ApplyThrust();
                ApplyGyros();
                ApplyDampeners();
                _hasOverrides = true;
                _writePhase++;
                _writeCount = _writeCountAccum;
            }

            // Drops every cached write value, so the next ApplyCommands pushes all of them.
            void InvalidateWriteCache()
            {
                for (int i = 0; i < _lastPct.Count; i++) _lastPct[i] = float.NaN;
                for (int i = 0; i < _gyros.Count && i < _lastGx.Count && i < _gyroOk.Count; i++)
                {
                    _lastGx[i] = float.NaN; _lastGy[i] = float.NaN; _lastGz[i] = float.NaN;
                }
                for (int i = 0; i < _lastGOn.Count; i++) _lastGOn[i] = false;
            }

            void ApplyThrust()
            {
                // Sign routes demand to a bucket: ThrottleForward +/- = forward/reverse, and so on.
                float pFwd = (float)Clamp01(_throttleForward);
                float pBack = (float)Clamp01(-_throttleForward);
                float pRight = (float)Clamp01(_strafeRight);
                float pLeft = (float)Clamp01(-_strafeRight);
                float pUp = (float)Clamp01(_strafeUp);
                float pDown = (float)Clamp01(-_strafeUp);

                int cyc = ForceWriteCycle > 0 ? ForceWriteCycle : 1;
                int phase = _writePhase % cyc;
                for (int i = 0; i < _thrusters.Count && i < _bucket.Count
                     && i < _lastPct.Count && i < _thrOk.Count; i++)
                {
                    // _thrOk is sampled on the same cadence as the thrust budgets, so the write gate and
                    // MaxForwardThrust agree on which thrusters are live. Nothing here touches a block
                    // until a write is actually due, so steady cruise costs no ModAPI calls at all.
                    // A thruster that stops working keeps whatever override it had; forget the cached
                    // value so it is re-pushed when the block comes back.
                    if (!_thrOk[i]) { _lastPct[i] = float.NaN; continue; }
                    float pct;
                    switch (_bucket[i])
                    {
                        case BFwd: pct = pFwd; break;
                        case BBack: pct = pBack; break;
                        case BRight: pct = pRight; break;
                        case BLeft: pct = pLeft; break;
                        case BUp: pct = pUp; break;
                        default: pct = pDown; break;
                    }
                    if (pct == _lastPct[i] && i % cyc != phase) continue;
                    IMyThrust t = _thrusters[i];
                    if (t == null || t.Closed) { _lastPct[i] = float.NaN; continue; }
                    // 0 is the sentinel that DISABLES the override, which is what lets SE's dampeners
                    // finish the stop once the closer asks for them.
                    t.ThrustOverridePercentage = pct;
                    _lastPct[i] = pct;
                    _writeCountAccum++;
                }
            }

            void ApplyGyros()
            {
                // GYRO UNITS: IMyGyro.Pitch/Yaw/Roll are ABSOLUTE rad/s, not a [-1,1] fraction of gyro
                // power. The [-1,1] command must be scaled by the rate cap, and the attitude law
                // normalises by the same number.
                double rateCap = GyroRateCap;
                Vector3D rotationBody = new Vector3D(_pitch, _yaw, _roll) * rateCap;
                Vector3D rotationWorld = Vector3D.TransformNormal(rotationBody, _refMatrix);

                int cyc = ForceWriteCycle > 0 ? ForceWriteCycle : 1;
                int phase = _writePhase % cyc;
                for (int i = 0; i < _gyros.Count && i < _lastGx.Count && i < _gyroOk.Count; i++)
                {
                    // Enabled/functional is sampled with the torque budget, same cadence, same reason.
                    if (!_gyroOk[i]) { _lastGx[i] = float.NaN; _lastGOn[i] = false; continue; }
                    IMyGyro g = _gyros[i];
                    if (g == null || g.Closed)
                    {
                        _lastGx[i] = float.NaN; _lastGOn[i] = false;
                        continue;
                    }
                    // Gyro local axes are not the reference frame's unless the gyro happens to be
                    // aligned, so convert through world space per gyro.
                    Vector3D gl = Vector3D.TransformNormal(rotationWorld, MatrixD.Transpose(g.WorldMatrix));
                    float gx = (float)gl.X, gy = (float)gl.Y, gz = (float)gl.Z;
                    bool force = i % cyc == phase;
                    // Compare what was actually WRITTEN, not what was intended: SE's Pitch/Yaw/Roll
                    // setters do not necessarily hand back the same number they were given.
                    if (!force && _lastGOn[i] && gx == _lastGx[i] && gy == _lastGy[i] && gz == _lastGz[i])
                        continue;
                    if (force || !_lastGOn[i]) { g.GyroOverride = true; _lastGOn[i] = true; _writeCountAccum++; }
                    if (force || gx != _lastGx[i]) { g.Pitch = gx; _lastGx[i] = gx; _writeCountAccum++; }
                    if (force || gy != _lastGy[i]) { g.Yaw = gy; _lastGy[i] = gy; _writeCountAccum++; }
                    if (force || gz != _lastGz[i]) { g.Roll = gz; _lastGz[i] = gz; _writeCountAccum++; }
                }
            }

            void ApplyDampeners()
            {
                if (_dampenersValid && _dampenersWanted == _dampenersApplied) return;
                for (int i = 0; i < _controllers.Count; i++)
                {
                    IMyShipController c = _controllers[i];
                    if (c == null || c.Closed) continue;
                    c.DampenersOverride = _dampenersWanted;
                    _writeCountAccum++;
                }
                _dampenersApplied = _dampenersWanted;
                _dampenersValid = true;
            }

            // Releases every override this instance set.
            public void ClearOverrides()
            {
                if (!_hasOverrides) return;
                for (int i = 0; i < _thrusters.Count; i++)
                {
                    IMyThrust t = _thrusters[i];
                    if (t != null && !t.Closed) t.ThrustOverride = 0f;
                }
                for (int i = 0; i < _gyros.Count; i++)
                {
                    IMyGyro g = _gyros[i];
                    if (g == null || g.Closed) continue;
                    // ORDER MATTERS. IMyGyro's Pitch/Yaw/Roll setters are wrapped in `if (GyroOverride)`
                    // (MyGyro.cs:141-189), so dropping the override first makes the three zeroing writes
                    // no-ops and the sliders keep the last commanded slew rate -- which is then applied
                    // in full the moment anything re-ticks Override, including a world reload.
                    g.Pitch = 0f; g.Yaw = 0f; g.Roll = 0f;
                    g.GyroOverride = false;
                }
                InvalidateWriteCache();
                _throttleForward = _strafeRight = _strafeUp = 0.0;
                _pitch = _yaw = _roll = 0.0;
                _dampenersWanted = false;
                // "Unknown", not false: the next ApplyDampeners must actually push a value.
                _dampenersValid = false;
                _hasOverrides = false;
            }

            // Newtons we are currently ASKING for, from the commands and the per-sign budgets. The fuel
            // estimator integrates this: summing IMyThrust.CurrentThrust would mean walking every
            // thruster every run, and this costs nothing.
            public double CommandedThrustNewtons
            {
                get
                {
                    double n = Math.Abs(_throttleForward) * (_throttleForward >= 0.0 ? _maxForwardThrust : _maxBackwardThrust);
                    n += Math.Abs(_strafeRight) * (_strafeRight >= 0.0 ? _maxRightThrust : _maxLeftThrust);
                    n += Math.Abs(_strafeUp) * (_strafeUp >= 0.0 ? _maxUpThrust : _maxDownThrust);
                    return n;
                }
            }

            public int ThrusterCount { get { return _thrusters.Count; } }
            public int GyroCount { get { return _gyros.Count; } }

            // Terminal-property writes actually issued by the last ApplyCommands. Steady cruise should
            // sit near the force-refresh floor; if it sits near thrusters + 4*gyros the cache is not
            // holding and something else is moving the overrides.
            public int LastWriteCount { get { return _writeCount; } }
            int _writeCount, _writeCountAccum;

            // Cost-probe entry points. They re-push the SAME values, so the ship is unaffected; the
            // caches are dropped first so what is priced is a real write, not a skipped one.
            // _hasOverrides is set because these DO claim the blocks: probing from Idle writes
            // GyroOverride = true at a zero rate, and without the flag ClearOverrides would refuse to
            // release them, leaving the ship unrotatable with no way back short of ticking every gyro
            // by hand.
            public void ProbeThrust() { InvalidateWriteCache(); ApplyThrust(); _hasOverrides = true; }
            public void ProbeGyros() { InvalidateWriteCache(); ApplyGyros(); _hasOverrides = true; }

            // The ModAPI READS a RefreshState does, with nothing stored. RefreshState itself must never
            // be called twice in one run: it would difference position against itself and zero the
            // derived velocity.
            public void ProbeReads()
            {
                if (_ctrl == null) return;
                _probeSink = _ctrl.CenterOfMass.X
                           + _ctrl.GetShipVelocities().LinearVelocity.X
                           + _ctrl.GetNaturalGravity().X
                           + _me.CubeGrid.WorldMatrix.Forward.X;
            }
            double _probeSink;

            static double InertiaAboutAxis(Vector3D axisGrid, Vector3D inertiaGrid)
            {
                return axisGrid.X * axisGrid.X * inertiaGrid.X
                     + axisGrid.Y * axisGrid.Y * inertiaGrid.Y
                     + axisGrid.Z * axisGrid.Z * inertiaGrid.Z;
            }

            static double Clamp01(double v) { return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v); }
            static double Clamp1(double v) { return v < -1.0 ? -1.0 : (v > 1.0 ? 1.0 : v); }
        }
    }
}

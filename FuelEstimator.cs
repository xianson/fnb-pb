using System;
using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    partial class Program
    {
        // Rough endurance guard. The mod has a real fuel guard that reserves brake authority; this is
        // deliberately only a guesstimate, but a guesstimate that is MEASURED rather than assumed.
        //
        // A PB cannot read a thruster's fuel consumption rate. So instead of hardcoding one, this
        // watches how fast the stores actually drain against how many newton-seconds we commanded,
        // and divides. Same approach as the gyro calibration: ask the ship, do not assume the ship.
        public sealed class FuelEstimator
        {
            readonly IMyGridTerminalSystem _gts;
            readonly IMyProgrammableBlock _me;

            readonly List<IMyGasTank> _tanks = new List<IMyGasTank>();
            readonly List<IMyBatteryBlock> _batteries = new List<IMyBatteryBlock>();
            readonly List<IMyReactor> _reactors = new List<IMyReactor>();

            // Live stores.
            public double H2Litres, H2Capacity;
            public double PowerMWh, PowerCapacityMWh;
            public bool HasH2, HasBattery, HasReactor;

            public double H2Frac { get { return H2Capacity > 1e-6 ? H2Litres / H2Capacity : 0.0; } }
            public double PowerFrac { get { return PowerCapacityMWh > 1e-6 ? PowerMWh / PowerCapacityMWh : 0.0; } }

            // Measured specific consumption, per newton-second of commanded thrust.
            double _specificH2;      // litres / (N*s)
            double _specificPwr;     // MWh    / (N*s)
            public bool Calibrated { get; private set; }

            // Sampling window.
            const int SampleIntervalRuns = 30;
            int _runs = int.MaxValue;
            double _winNewtonSeconds;
            double _winStartH2, _winStartPwr;
            bool _winOpen;
            // Ignore windows with negligible thrust: dividing a tiny fuel delta by a tiny impulse is
            // all noise, and a recharging battery makes the delta negative.
            public double MinWindowNewtonSeconds = 1e5;
            // Low-pass on the specific figures so one odd window cannot dominate.
            public double Smooth = 0.3;

            public FuelEstimator(IMyGridTerminalSystem gts, IMyProgrammableBlock me)
            {
                _gts = gts;
                _me = me;
            }

            public void Rescan()
            {
                _tanks.Clear();
                _batteries.Clear();
                _reactors.Clear();
                _gts.GetBlocksOfType<IMyGasTank>(_tanks, b => b.IsSameConstructAs(_me) && IsHydrogen(b));
                _gts.GetBlocksOfType<IMyBatteryBlock>(_batteries, b => b.IsSameConstructAs(_me));
                _gts.GetBlocksOfType<IMyReactor>(_reactors, b => b.IsSameConstructAs(_me));
                HasH2 = _tanks.Count > 0;
                HasBattery = _batteries.Count > 0;
                HasReactor = _reactors.Count > 0;
                _winOpen = false;
            }

            // Hydrogen tanks are MyObjectBuilder_GasTank; oxygen tanks are MyObjectBuilder_OxygenTank.
            // Subtype is checked too so modded hydrogen tanks are still caught.
            static bool IsHydrogen(IMyGasTank t)
            {
                string type = t.BlockDefinition.TypeIdString ?? "";
                string sub = t.BlockDefinition.SubtypeName ?? "";
                if (sub.IndexOf("Hydrogen", StringComparison.OrdinalIgnoreCase) >= 0) return true;
                return type.IndexOf("Oxygen", StringComparison.OrdinalIgnoreCase) < 0;
            }

            // newtonSeconds is accumulated by the caller from COMMANDED thrust, which costs nothing --
            // summing IMyThrust.CurrentThrust every run would mean iterating every thruster.
            public void Update(double dt, double commandedNewtons)
            {
                _winNewtonSeconds += commandedNewtons * dt;

                if (_runs++ < SampleIntervalRuns) return;
                _runs = 0;

                ReadStores();

                if (!_winOpen)
                {
                    OpenWindow();
                    return;
                }

                if (_winNewtonSeconds >= MinWindowNewtonSeconds)
                {
                    double dH2 = _winStartH2 - H2Litres;
                    double dPwr = _winStartPwr - PowerMWh;
                    if (dH2 > 0.0 && HasH2)
                    {
                        double s = dH2 / _winNewtonSeconds;
                        _specificH2 = Calibrated ? _specificH2 + (s - _specificH2) * Smooth : s;
                        Calibrated = true;
                    }
                    if (dPwr > 0.0 && HasBattery)
                    {
                        double s = dPwr / _winNewtonSeconds;
                        _specificPwr = Calibrated ? _specificPwr + (s - _specificPwr) * Smooth : s;
                        Calibrated = true;
                    }
                    OpenWindow();
                }
                else if (_winNewtonSeconds <= 0.0)
                {
                    OpenWindow();   // idle window, nothing to learn
                }
            }

            void OpenWindow()
            {
                _winStartH2 = H2Litres;
                _winStartPwr = PowerMWh;
                _winNewtonSeconds = 0.0;
                _winOpen = true;
            }

            void ReadStores()
            {
                H2Litres = 0.0; H2Capacity = 0.0;
                for (int i = 0; i < _tanks.Count; i++)
                {
                    IMyGasTank t = _tanks[i];
                    if (t == null || t.Closed || !t.IsFunctional) continue;
                    H2Capacity += t.Capacity;
                    H2Litres += t.Capacity * t.FilledRatio;
                }
                PowerMWh = 0.0; PowerCapacityMWh = 0.0;
                for (int i = 0; i < _batteries.Count; i++)
                {
                    IMyBatteryBlock b = _batteries[i];
                    if (b == null || b.Closed || !b.IsFunctional) continue;
                    PowerCapacityMWh += b.MaxStoredPower;
                    PowerMWh += b.CurrentStoredPower;
                }
                RefreshReactorLive();
            }

            // Seconds of continuous MAIN-DRIVE burn the stores support. Returns -1 when unknown.
            // Whichever store runs out first wins; a working reactor is treated as removing the
            // electrical limit, since its uranium draw is negligible next to thruster fuel.
            public double BurnSecondsRemaining(double maxForwardThrustN)
            {
                if (!Calibrated || maxForwardThrustN < 1e-6) return -1.0;
                double best = double.PositiveInfinity;
                if (HasH2 && _specificH2 > 1e-18)
                    best = Math.Min(best, H2Litres / (_specificH2 * maxForwardThrustN));
                if (HasBattery && _specificPwr > 1e-18 && !ReactorLive())
                    best = Math.Min(best, PowerMWh / (_specificPwr * maxForwardThrustN));
                return double.IsInfinity(best) ? -1.0 : best;
            }

            // Sampled with the stores, not per call: BurnSecondsRemaining is asked twice per run and
            // this walked every reactor block each time.
            bool _reactorLive;

            bool ReactorLive() { return _reactorLive; }

            void RefreshReactorLive()
            {
                _reactorLive = false;
                for (int i = 0; i < _reactors.Count; i++)
                {
                    IMyReactor r = _reactors[i];
                    if (r != null && !r.Closed && r.IsWorking) { _reactorLive = true; return; }
                }
            }
        }
    }
}

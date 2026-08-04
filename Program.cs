using System;
using System.Collections.Generic;
using System.Text;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    // FlipAndBurn autopilot, Programmable Block port.
    //
    // Control laws are ported from D:\fnb-merge\mods\FlipAndBurn\Data\Scripts\FlipAndBurn\WaypointQueue.
    // A PB cannot create terminal controls, so the mod's FB_/AutoRotate_ properties are exposed as
    // key = value lines in this block's CustomData. See README.md.
    partial class Program : MyGridProgram
    {
        const string StatusMarker = "; ---- status (written by the script) ----";

        // MDK strips usings, so System.Globalization must be spelled out here. Invariant culture is
        // mandatory: under a comma-decimal locale "123.45" would otherwise parse as 123450.
        // IFormatProvider itself is not whitelisted, so the field must be typed as CultureInfo.
        static readonly System.Globalization.CultureInfo Inv =
            System.Globalization.CultureInfo.InvariantCulture;

        static bool TryParseD(string s, out double v)
        {
            return double.TryParse(s, System.Globalization.NumberStyles.Float, Inv, out v);
        }

        static bool TryParseI(string s, out int v)
        {
            return int.TryParse(s, System.Globalization.NumberStyles.Integer, Inv, out v);
        }

        enum ApState { Idle, Goto, Align, Arrived, Fault, Calibrate }

        // MINIFIED ENUM NAMES. MDK's minifier renames enum members, so Enum.ToString() in the packed
        // build prints the mangled identifier -- that is the mojibake on the state line. It is not
        // cosmetic either: AutoRotate_Mode is written to CustomData and read back through
        // AlignController.TryParse, which cannot match a mangled name. Every enum that reaches Echo or
        // CustomData is mapped to a literal below.
        static string StateName(ApState s)
        {
            switch (s)
            {
                case ApState.Idle: return "Idle";
                case ApState.Goto: return "Goto";
                case ApState.Align: return "Align";
                case ApState.Arrived: return "Arrived";
                case ApState.Fault: return "Fault";
                case ApState.Calibrate: return "Calibrate";
            }
            return "?";
        }


        PbShip _ship;
        OrientationController _att;
        AlignController _align;
        // The flown plan: Stop-to-Stop sub-missions, each a Catmull-Rom spline flown by QtrtController
        // and closed by FineTranslationController. A bare `goto` is a one-leg route.
        WaypointQueueController _queue;
        WaypointEntry[] _pendingRoute;
        string _routeText = "";
        GyroCalibrator _cal;
        FuelEstimator _fuel;
        int _fuelScanCountdown;
        string _fuelMargin = "unknown";

        ApState _state = ApState.Idle;
        Vector3D _targetCoord;
        bool _hasTarget;
        string _targetName = "";
        string _fault = "";

        // Mirrors of the CustomData input fields.
        RotationMode _wantMode = RotationMode.None;
        bool _wantEngage;
        string _controllerHint = "";
        double _cfgGyroRateCap = OrientationController.CommandedRateCapBase;
        double _cfgArrivalDist = -1.0;
        double _cfgGyroTorque;   // 0 = measure it
        double _tackDeg = 30.0;  // cant used by the tack command
        double _tackBias;        // default bias when the tack argument omits one
        bool _pendingGoto;

        string _lastCustomData = "\u0000";   // force a parse on the first run
        int _statusCountdown;
        double _peakInstrFrac;
        int _lastInstr;
        double _peakRunMs;
        double _lastRunMs;

        // SIM SPEED. A PB cannot read it, so infer it: the scheduler fires every N game-ticks, and
        // TimeSinceLastRun measures the gap. Under load the server runs fewer ticks per real second,
        // so the ratio of expected-to-observed is the sim speed. Heavily smoothed -- a single run is
        // noise. Reads 1.00 flat if TimeSinceLastRun turns out to be game time on this build.
        double _simSpeed = 1.0;
        bool _simSpeedValid;

        // Reading Me.CustomData means a string compare against the ~1.5 kB we last wrote, and writing
        // it is a synced string set. Neither needs to happen at 60 Hz -- these are human-edited fields.
        public int CustomDataPollRuns = 10;
        int _cdPollCountdown;

        // The Echo block formats ~20 doubles and concatenates ~25 strings. SE clears the echo buffer
        // between runs so the text must be pushed every run, but it only has to be BUILT at a rate a
        // human can read; the rest of the time the cached string is re-emitted in one call.
        public int EchoIntervalRuns = 10;
        int _echoCountdown;
        string _echoText = "FlipAndBurn PB";

        // PEAK ATTRIBUTION. Runtime.LastRunTimeMs read inside Main is the PREVIOUS run's duration, so
        // the peak has to be credited to the previous run's tag. Without this a one-off spike is
        // indistinguishable from a steady cost, and first-run JIT is indistinguishable from the bake.
        string _runTag = "ctor";
        string _prevTag = "ctor";
        string _peakTag = "";
        int _runCount;

        // Only a scheduled run has a known cadence to compare against; a terminal/trigger run does not.
        void TrackSimSpeed(UpdateType src)
        {
            double expected =
                (src & UpdateType.Update1) != 0 ? 1.0 / 60.0 :
                (src & UpdateType.Update10) != 0 ? 10.0 / 60.0 :
                (src & UpdateType.Update100) != 0 ? 100.0 / 60.0 : 0.0;
            if (expected <= 0.0) return;

            double real = Runtime.TimeSinceLastRun.TotalSeconds;
            if (real <= 1e-6) return;

            double s = expected / real;
            if (s > 1.5) s = 1.5;
            if (s < 0.05) s = 0.05;
            _simSpeed = _simSpeedValid ? _simSpeed + (s - _simSpeed) * 0.05 : s;
            _simSpeedValid = true;
        }

        // Wall-clock seconds for a duration the plan measures in sim seconds.
        double RealSeconds(double simSeconds)
        {
            double s = _simSpeedValid && _simSpeed > 0.05 ? _simSpeed : 1.0;
            return simSeconds / s;
        }

        void Tag(string t)
        {
            _runTag = _runTag == "tick" || _runTag == "firstrun" ? t : _runTag + "+" + t;
        }

        public Program()
        {
            _ship = new PbShip(GridTerminalSystem, Me, Echo);
            _att = new OrientationController(_ship);
            _align = new AlignController(_ship, _att);
            _fuel = new FuelEstimator(GridTerminalSystem, Me);
            _fuel.Rescan();
            LoadStorage();
            Runtime.UpdateFrequency = UpdateFrequency.Update100;
        }

        public void Save()
        {
            Storage = string.Join("\n",
                ((int)_state).ToString(Inv),
                Fmt(_targetCoord),
                _hasTarget ? "1" : "0",
                ((int)_align.Mode).ToString(Inv));
        }

        void LoadStorage()
        {
            if (string.IsNullOrEmpty(Storage)) return;
            string[] p = Storage.Split('\n');
            if (p.Length < 4) return;
            int st;
            if (TryParseI(p[0], out st))
                _state = (ApState)st;
            Vector3D c;
            if (TryParseVector(p[1], out c)) _targetCoord = c;
            _hasTarget = p[2] == "1";
            int m;
            if (TryParseI(p[3], out m))
                _align.Mode = (RotationMode)m;
            // A saved Goto cannot resume mid-profile: the closer's phase state is not persisted.
            if (_state == ApState.Goto) _state = ApState.Idle;
        }

        public void Main(string argument, UpdateType updateSource)
        {
            _prevTag = _runTag;
            _runCount++;
            _runTag = _runCount == 1 ? "firstrun" : "tick";

            double dt = Runtime.TimeSinceLastRun.TotalSeconds;
            if (dt <= 0.0 || dt > 5.0) dt = 1.0 / 60.0;

            TrackSimSpeed(updateSource);

            if (!string.IsNullOrEmpty(argument))
            {
                Tag("arg");
                HandleArgument(argument);
            }

            if (--_cdPollCountdown <= 0)
            {
                _cdPollCountdown = CustomDataPollRuns > 0 ? CustomDataPollRuns : 1;
                SyncCustomData();
            }

            Tick(dt);

            if (--_fuelScanCountdown <= 0) { _fuelScanCountdown = 600; Tag("fuelscan"); _fuel.Rescan(); }
            _fuel.Update(dt, _ship.CommandedThrustNewtons);
            TickFuelGuard();

            WriteStatus();
            RunProbe();

            if (--_echoCountdown <= 0)
            {
                _echoCountdown = EchoIntervalRuns > 0 ? EchoIntervalRuns : 1;
                UpdateFuelMargin();
                TrackPerf();
                _echoText = BuildEcho();
            }
            else TrackPerf();
            Echo(_echoText);
        }

        // Compares measured endurance against the burn the current plan still needs. Advisory only --
        // it does NOT abort a run, because cutting thrust mid-brake is worse than arriving dry.
        // The closer for the leg being flown, or null while the spline tracker still owns the ship.
        FineTranslationController ActiveFine()
        {
            SubMission s = _queue == null ? null : _queue.CurrentSub;
            return s == null ? null : s.Fine;
        }

        // Brake-fuel guard. Mirrors the mod's TickFuelGuard: latched, escalate-only, two stages --
        // coast (stop accelerating) then force-brake. The mod compares fuel UNITS from the grid API;
        // a PB has no such API, so both sides are expressed as seconds of full main-drive burn, which
        // is what FuelEstimator measures. Margins and the brake model are the mod's.
        const double FuelGuardCoastMargin = 1.15;
        const double FuelGuardForceMargin = 1.02;
        const double FuelGuardFlipOverheadX = 5.0;
        int _fuelGuardStage;   // 0 none, 1 coast, 2 force-brake (latched per route)

        void TickFuelGuard()
        {
            if (_queue == null || _state != ApState.Goto) return;
            SubMission sub = _queue.CurrentSub;
            if (sub == null || sub.Qtrt == null) return;

            double have = _fuel.BurnSecondsRemaining(_ship.MaxForwardThrust);
            if (have < 0.0) return;                    // not calibrated yet -> inert

            IRigidBody rb = _ship.Body;
            double v = rb.LinearVelocity.Length();
            double aMaxFwd = _ship.MaxForwardThrust * rb.InvMass;
            if (aMaxFwd < 1e-6 || v < 1.0) return;
            // Gravity along the travel direction eats into the brake authority.
            double gAlong = Vector3D.Dot(_ship.Gravity, rb.LinearVelocity / v);
            double aG = Math.Max(0.1, aMaxFwd - Math.Max(0.0, gAlong));
            double need = v / aG
                        + FuelGuardFlipOverheadX * OrientationController.DeriveFlipLeadTime(_ship);

            int stage = 0;
            if (have < need * FuelGuardForceMargin) stage = 2;
            else if (have < need * FuelGuardCoastMargin) stage = 1;
            if (stage <= _fuelGuardStage) return;      // latched: escalate only

            _fuelGuardStage = stage;
            sub.Qtrt.FuelCoast = true;
            if (stage >= 2) sub.Qtrt.FuelForceBrake = true;
        }

        void UpdateFuelMargin()
        {
            double have = _fuel.BurnSecondsRemaining(_ship.MaxForwardThrust);
            if (have < 0.0) { _fuelMargin = _fuel.Calibrated ? "n/a" : "measuring"; return; }
            FineTranslationController fine = ActiveFine();
            if (fine == null || _state != ApState.Goto)
            {
                _fuelMargin = have.ToString("0", Inv) + "s burn left";
                return;
            }
            double need = fine.RequiredBurnSeconds();
            if (need < 0.0) { _fuelMargin = "n/a"; return; }
            double ratio = need > 1e-6 ? have / need : 999.0;
            string tag = ratio >= 1.5 ? "OK" : (ratio >= 1.0 ? "MARGINAL" : "INSUFFICIENT");
            _fuelMargin = tag + " (" + have.ToString("0", Inv)
                        + "s have / " + need.ToString("0", Inv) + "s need)";
        }

        // ---------- Command surface ----------

        void HandleArgument(string arg)
        {
            string a = arg.Trim();
            int sp = a.IndexOf(' ');
            string verb = (sp < 0 ? a : a.Substring(0, sp)).ToLowerInvariant();
            string rest = sp < 0 ? "" : a.Substring(sp + 1).Trim();

            switch (verb)
            {
                case "goto":
                case "tack":
                    {
                        // tack flies the identical route; it only holds the drive off the track so
                        // Spectrum's drive lobe never points down it. See QtrtController.TackAngleRad.
                        QtrtController.TackAngleRad = verb == "tack" ? _tackDeg * Math.PI / 180.0 : 0.0;
                        if (verb == "tack") QtrtController.TackBias = StripTackBias(ref rest);
                        if (rest.Length > 0)
                        {
                            Vector3D c;
                            if (!TryParseDestination(rest, out c)) { _fault = verb + ": bad coordinate"; return; }
                            _targetCoord = c;
                            _targetName = DestinationName(rest);
                            _hasTarget = true;
                        }
                        if (!_hasTarget) { _fault = verb + ": no target set"; return; }
                        EngageGoto();
                        break;
                    }
                case "route":
                    {
                        QtrtController.TackAngleRad = 0.0;
                        if (rest.Length == 0) { _fault = "route: no waypoints"; return; }
                        WaypointEntry[] wps;
                        string err;
                        if (!TryParseRoute(rest, out wps, out err)) { _fault = "route: " + err; return; }
                        _routeText = rest;
                        EngageRoute(wps);
                        break;
                    }
                case "align":
                    {
                        RotationMode m;
                        if (!AlignController.TryParse(rest, out m)) { _fault = "align: bad mode"; return; }
                        EngageAlign(m);
                        break;
                    }
                case "stop":
                case "abort":
                case "off":
                    Disengage("commanded");
                    break;
                case "target":
                    {
                        Vector3D c;
                        if (!TryParseDestination(rest, out c)) { _fault = "target: bad coordinate"; return; }
                        _targetCoord = c;
                        _targetName = DestinationName(rest);
                        _hasTarget = true;
                        break;
                    }
                case "calibrate":
                    // Spins the ship. Refuse mid-mission rather than corrupting a run in progress.
                    if (_state == ApState.Goto) { _fault = "calibrate: stop the run first"; return; }
                    _ship.ClearOverrides();
                    _align.Mode = RotationMode.None;
                    _wantMode = RotationMode.None;
                    _att.ResetTracking();
                    _queue = null; _pendingRoute = null;
                    _pendingGoto = false;
                    _cal = new GyroCalibrator(_ship);
                    _state = ApState.Calibrate;
                    _fault = "";
                    Runtime.UpdateFrequency = ControlFrequency();
                    break;
                case "resetperf":
                    // Peaks are sticky by design, so there has to be a way to clear them after a fix.
                    _peakInstrFrac = 0.0;
                    _peakRunMs = 0.0;
                    _peakTag = "";
                    break;
                case "probe":
                    {
                        int sp2 = rest.IndexOf(' ');
                        string what = (sp2 < 0 ? rest : rest.Substring(0, sp2)).Trim().ToLowerInvariant();
                        int n = 100;
                        if (sp2 >= 0) TryParseI(rest.Substring(sp2 + 1).Trim(), out n);
                        if (what.Length == 0) { _fault = "probe: name a target"; return; }
                        StartProbe(what, n);
                        break;
                    }
                case "rescan":
                    _ship.ClearOverrides();
                    _ship = new PbShip(GridTerminalSystem, Me, Echo);
                    _att = new OrientationController(_ship);
                    _align = new AlignController(_ship, _att);
                    ApplyConfigToShip();
                    _state = ApState.Idle;
                    break;
                default:
                    _fault = "unknown argument: " + verb;
                    break;
            }
        }

        // Deliberately does NOT refresh ship state: RefreshState samples position for the velocity
        // estimate, and calling it twice in one run differences a position against itself and yields
        // a velocity of zero. Validation happens in Tick, on the single refresh per run.
        // A bare goto is a one-leg route: the queue prepends an implicit Stop at the ship, so this
        // becomes a single Stop-to-Stop sub-mission flown by the same spline law as any other leg.
        void EngageGoto()
        {
            _routeText = "";
            EngageRoute(new WaypointEntry[] { WaypointEntry.Stop(_targetCoord) });
        }

        void EngageRoute(WaypointEntry[] wps)
        {
            _fault = "";
            _att.ResetTracking();
            _align.Mode = RotationMode.None;
            _wantMode = RotationMode.None;
            _queue = null;
            _pendingRoute = wps;
            _fuelGuardStage = 0;
            // The schedule is built from velocity at construction, and the derived velocity needs at
            // least two position samples. Building it from the SE-clamped physical velocity while
            // HighSpeed is engaged would understate the brake distance ~100x, so wait for a real read.
            _pendingGoto = true;
            _wantEngage = true;
            _state = ApState.Goto;
            Runtime.UpdateFrequency = ControlFrequency();
        }

        UpdateFrequency ControlFrequency()
        {
            // Always per-tick. Update10 bought performance by coarsening the attitude law's
            // switching decision; the script no longer needs that trade.
            return UpdateFrequency.Update1;
        }

        void EngageAlign(RotationMode m)
        {
            _fault = "";
            if (m == RotationMode.None) { Disengage("align off"); return; }
            _ship.ClearOverrides();
            _att.ResetTracking();
            _align.Reset();
            _align.Mode = m;
            _wantMode = m;
            _queue = null; _pendingRoute = null;
            _pendingGoto = false;
            _state = ApState.Align;
            Runtime.UpdateFrequency = ControlFrequency();
        }

        void Disengage(string why)
        {
            _ship.ClearOverrides();
            _align.Mode = RotationMode.None;
            _align.Reset();
            _queue = null; _pendingRoute = null;
            _pendingGoto = false;
            _wantEngage = false;
            _wantMode = RotationMode.None;
            _state = ApState.Idle;
            _fault = "";
            Runtime.UpdateFrequency = UpdateFrequency.Update100;
        }

        // ---------- Control loop ----------

        void Tick(double dt)
        {
            if (_state == ApState.Idle || _state == ApState.Fault || _state == ApState.Arrived)
                return;

            if (_ship.WillRescan) Tag("rescan");
            if (!_ship.RefreshState(dt))
            {
                _fault = "lost ship controller";
                _ship.ClearOverrides();
                _state = ApState.Fault;
                Runtime.UpdateFrequency = UpdateFrequency.Update100;
                return;
            }

            // The min-time attitude law's zero-order hold is the period its command is actually held
            // for, which in a PB is the run interval rather than the engine tick.
            _att.ControlPeriod = dt;

            // GPS/Target alignment aims at FB_TargetCoord, the same field GoTo flies to.
            _align.GpsTarget = _targetCoord;
            _align.HasGpsTarget = _hasTarget;

            if (_state == ApState.Goto)
            {
                if (_pendingGoto)
                {
                    if (!(_ship.MaxForwardThrust > 1e-6))
                    {
                        _fault = "no main-drive thrust";
                        _ship.ClearOverrides();
                        _state = ApState.Fault;
                        Runtime.UpdateFrequency = UpdateFrequency.Update100;
                        return;
                    }
                    if (!_ship.DerivedVelocityValid) return;   // hold, hands off, until velocity is real
                    try
                    {
                        Tag("engage");
                        _queue = new WaypointQueueController(_ship, _att, _pendingRoute);
                    }
                    catch (Exception e)
                    {
                        _fault = "route rejected: " + e.Message;
                        _ship.ClearOverrides();
                        _state = ApState.Fault;
                        Runtime.UpdateFrequency = UpdateFrequency.Update100;
                        return;
                    }
                    _pendingGoto = false;
                }
                _queue.Update(dt);
                if (_queue.Planning) Tag("plan");
                // A new leg starts with a clean fuel-guard latch, as the mod's driver does.
                if (_queue.JustAdvancedSubMission)
                {
                    _queue.JustAdvancedSubMission = false;
                    _fuelGuardStage = 0;
                    SubMission adv = _queue.CurrentSub;
                    if (adv != null && adv.Qtrt != null)
                    {
                        adv.Qtrt.FuelCoast = false;
                        adv.Qtrt.FuelForceBrake = false;
                    }
                }
                _ship.ApplyCommands();
                if (_queue.IsDone)
                {
                    // The closer has already asked for dampeners; push that, then release the gyro and
                    // thrust overrides so SE's own damping finishes the stop and the pilot has control.
                    _ship.ClearOverrides();
                    _state = ApState.Arrived;
                    Runtime.UpdateFrequency = UpdateFrequency.Update100;
                }
                return;
            }

            if (_state == ApState.Calibrate)
            {
                _cal.Update(dt);
                _ship.ApplyCommands();
                if (_cal.IsDone)
                {
                    // Persist into CustomData, not just memory: it is the block's own interface, so
                    // the numbers survive a recompile and are visible and editable afterwards.
                    if (_cal.ResultTorque > 0.0) _cfgGyroTorque = _cal.ResultTorque;
                    if (_cal.RateCapValid) _cfgGyroRateCap = _cal.ResultRateCap;
                    ApplyConfigToShip();
                    _ship.ClearOverrides();
                    _state = ApState.Idle;
                    _statusCountdown = 0;   // write the results out immediately
                    Runtime.UpdateFrequency = UpdateFrequency.Update100;
                }
                return;
            }

            if (_state == ApState.Align)
            {
                _ship.ThrottleForward = 0.0;
                _ship.StrafeRight = 0.0;
                _ship.StrafeUp = 0.0;
                // No usable reference (Prograde at rest, Gravity in space, GPS with no target): release
                // rather than hold a zero-rate override, which would otherwise lock out the pilot.
                if (!_align.Update()) _ship.ClearOverrides();
                else _ship.ApplyCommands();
            }
        }

        // ---------- CustomData ----------

        void SyncCustomData()
        {
            string cd = Me.CustomData;
            if (cd == _lastCustomData) return;   // our own status write, or nothing changed
            ParseCustomData(cd);
            ApplyConfigToShip();
            _statusCountdown = 0;                // re-emit immediately so the user sees the effect
        }

        void ParseCustomData(string cd)
        {
            if (string.IsNullOrEmpty(cd)) return;
            string[] lines = cd.Split('\n');
            RotationMode parsedMode = RotationMode.None;
            bool sawMode = false;
            bool engage = false;

            for (int i = 0; i < lines.Length; i++)
            {
                string line = lines[i].Trim();
                if (line.Length == 0) continue;
                if (line.StartsWith(";") || line.StartsWith("#")) continue;
                if (line.StartsWith(StatusMarker.Substring(0, 6))) break;   // status section: input ends here
                int eq = line.IndexOf('=');
                if (eq <= 0) continue;
                string key = line.Substring(0, eq).Trim();
                string val = line.Substring(eq + 1).Trim();

                switch (key.ToLowerInvariant())
                {
                    case "fb_targetcoord":
                        {
                            Vector3D c;
                            if (val.Length > 0 && TryParseVector(val, out c))
                            {
                                _targetCoord = c;
                                _targetName = ExtractGpsName(val);
                                _hasTarget = true;
                            }
                            break;
                        }
                    case "fb_route":
                        // Edge-triggered, like the mode field: a changed route re-plans on the spot.
                        if (val != _routeText)
                        {
                            _routeText = val;
                            if (val.Length > 0)
                            {
                                WaypointEntry[] wps; string rerr;
                                if (TryParseRoute(val, out wps, out rerr)) EngageRoute(wps);
                                else _fault = "route: " + rerr;
                            }
                        }
                        break;
                    case "autorotate_mode":
                        sawMode = AlignController.TryParse(val, out parsedMode);
                        break;
                    case "fb_engage":
                        engage = val.Equals("true", StringComparison.OrdinalIgnoreCase) || val == "1";
                        break;
                    case "controller":
                        _controllerHint = val;
                        break;
                    case "gyroratecap":
                        double g;
                        if (TryParseD(val, out g) && g > 0.0)
                            _cfgGyroRateCap = g;
                        break;
                    case "gyrotorque":
                        double gt;
                        if (TryParseD(val, out gt) && gt >= 0.0)
                            _cfgGyroTorque = gt;
                        break;
                    case "arrivaldist":
                        double ad;
                        if (TryParseD(val, out ad) && ad > 0.0)
                            _cfgArrivalDist = ad;
                        break;
                    case "splinesamples":
                        // CHANGES FLIGHT. 48 is the mod's value and the only mod-identical setting;
                        // lowering it coarsens the speed schedule to buy instruction budget.
                        int ss;
                        if (TryParseI(val, out ss) && ss >= 8 && ss <= 64)
                            QtrtController.SamplesPerSegment = ss;
                        break;
                    case "tackangle":
                        // Cant used by the tack command. Past ~45 the emission lobe is already flat
                        // and the propellant curve is not, so the useful band is narrow.
                        double td;
                        if (TryParseD(val, out td) && td >= 0.0 && td <= 60.0)
                            _tackDeg = td;
                        break;
                    case "tackcone":
                        double tc;
                        if (TryParseD(val, out tc) && tc >= 5.0)
                            QtrtController.TackConeSeconds = tc;
                        break;
                    case "tackbias":
                        double tb;
                        if (TryParseD(val, out tb) && tb >= 0.0 && tb <= 1.0)
                            _tackBias = tb;
                        break;
                }
            }

            // Edge-triggered: a mode change in CustomData engages/releases alignment.
            if (sawMode && parsedMode != _wantMode)
            {
                _wantMode = parsedMode;
                if (_state != ApState.Goto)
                {
                    if (parsedMode == RotationMode.None) Disengage("align cleared");
                    else EngageAlign(parsedMode);
                }
            }

            if (engage != _wantEngage)
            {
                _wantEngage = engage;
                if (engage && _hasTarget) EngageGoto();
                else if (!engage && _state == ApState.Goto) Disengage("engage cleared");
            }
        }

        void ApplyConfigToShip()
        {
            _ship.ControllerNameHint = _controllerHint;
            _ship.GyroRateCapOverride = _cfgGyroRateCap;
            _ship.GyroTorqueOverride = _cfgGyroTorque;
            if (_state == ApState.Goto || _state == ApState.Align)
                Runtime.UpdateFrequency = ControlFrequency();
        }

        void WriteStatus()
        {
            if (_statusCountdown-- > 0) return;
            _statusCountdown = _state == ApState.Idle || _state == ApState.Arrived ? 30 : 60;

            StringBuilder sb = new StringBuilder();
            sb.Append("; FlipAndBurn autopilot (PB). Edit the fields below, then recompile or run.\n");
            sb.Append("; Arguments: goto <gps|x,y,z> | route <wp>;<wp>;... | align <mode> | stop\n");
            sb.Append("; route waypoint: [stop|spline|fly][@tol] <gps|x,y,z>; last one defaults to stop\n");
            sb.Append("FB_TargetCoord = ").Append(_hasTarget ? Fmt(_targetCoord) : "").Append('\n');
            sb.Append("FB_Route = ").Append(_routeText).Append('\n');
            sb.Append("AutoRotate_Mode = ").Append(ModeName(_align.Mode)).Append('\n');
            sb.Append("FB_Engage = ").Append(_state == ApState.Goto ? "true" : "false").Append('\n');
            sb.Append("Controller = ").Append(_controllerHint).Append('\n');
            sb.Append("GyroRateCap = ").Append(_cfgGyroRateCap.ToString("0.###", Inv)).Append('\n');
            sb.Append("SplineSamples = ")
              .Append(QtrtController.SamplesPerSegment.ToString(Inv)).Append('\n');
            sb.Append("GyroTorque = ").Append(_cfgGyroTorque > 0.0
                ? _cfgGyroTorque.ToString("0", Inv) : "auto").Append('\n');
            if (_cfgArrivalDist > 0.0)
                sb.Append("ArrivalDist = ").Append(_cfgArrivalDist.ToString("0.#", Inv)).Append('\n');
            sb.Append('\n');
            sb.Append(StatusMarker).Append('\n');
            AppendStatus(sb);

            string s = sb.ToString();
            // Writing CustomData is a synced string set plus a terminal-property-changed notification.
            // Compare against what the BLOCK holds, not against what we last wrote: comparing against
            // our own copy would leave a user edit in place forever and re-parse it every poll.
            _lastCustomData = s;
            if (s == Me.CustomData) return;
            Tag("status");
            Me.CustomData = s;
        }

        void AppendStatus(StringBuilder sb)
        {
            sb.Append("Autopilot_State = ").Append(StateName(_state)).Append('\n');
            if (_queue != null && _queue.Planning)
                sb.Append("Autopilot_Planning = ").Append(_queue.PlanWorkLeft.ToString(Inv)).Append('\n');
            if (_fault.Length > 0) sb.Append("Fault = ").Append(_fault).Append('\n');
            sb.Append("AutoRotate_Aligned = ").Append(_align.IsAligned ? "true" : "false").Append('\n');
            sb.Append("AutoRotate_Angle = ").Append(_align.AngleDeg.ToString("0.0", Inv)).Append('\n');

            IRigidBody rb = _ship.Body;
            double spd = rb.LinearVelocity.Length();
            sb.Append("Speed = ").Append(spd.ToString("0.0", Inv))
              .Append(_ship.UsingModVelocity ? " (FB_Velocity)" : " (derived)").Append('\n');
            sb.Append("PhysicalSpeed = ").Append(rb.PhysicalLinearVelocity.Length().ToString("0.0", Inv)).Append('\n');
            if (_hasTarget)
            {
                double rem = (_targetCoord - rb.Position).Length();
                sb.Append("Autopilot_RemainingDistance = ").Append(rem.ToString("0", Inv)).Append('\n');
                if (_targetName.Length > 0)
                    sb.Append("Autopilot_TargetName = ").Append(_targetName).Append('\n');
            }
            if (_queue != null)
            {
                SubMission sub = _queue.CurrentSub;
                sb.Append("Leg = ").Append((_queue.CurrentSubMission + 1).ToString(Inv))
                  .Append('/').Append(_queue.SubMissionCount.ToString(Inv)).Append('\n');
                if (sub != null)
                {
                    sb.Append("Phase = ").Append(sub.PhaseLabel()).Append('\n');
                    QtrtController q = sub.Qtrt;
                    if (q != null && sub.Fine == null)
                    {
                        // The tracker's own state: what it is chasing and how far off the path it is.
                        sb.Append("Sched_Speed = ").Append(q.DbgVSched.ToString("0.0", Inv)).Append('\n');
                        sb.Append("Along_Speed = ").Append(q.AlongTrackSpeed.ToString("0.0", Inv)).Append('\n');
                        sb.Append("CrossTrack = ").Append(q.DbgCross.ToString("0.0", Inv)).Append('\n');
                        sb.Append("AlignErr = ")
                          .Append((q.DbgAlignErr * 180.0 / Math.PI).ToString("0.0", Inv)).Append('\n');
                        if (q.DbgStraightRun != 0) sb.Append("StraightRun = true\n");
                        if (q.TermFlipDone) sb.Append("BrakeCommitted = true\n");
                    }
                    FineTranslationController fine = sub.Fine;
                    if (fine != null && fine.IsDone && fine.ArrivalTime >= 0.0)
                    {
                        sb.Append("ArrivalError = ")
                          .Append(Math.Sqrt(fine.ArrivalAlongTrackError * fine.ArrivalAlongTrackError
                                          + fine.ArrivalCrossTrackError * fine.ArrivalCrossTrackError)
                                  .ToString("0.0", Inv)).Append('\n');
                    }
                }
                double eta = _queue.EstimatedSecondsRemaining;
                sb.Append("Autopilot_ETA = ").Append(eta.ToString("0", Inv)).Append('\n');
                sb.Append("Autopilot_ETA_Real = ").Append(RealSeconds(eta).ToString("0", Inv)).Append('\n');
            }
            sb.Append("Sim_Speed = ").Append(_simSpeed.ToString("0.00", Inv)).Append('\n');
            if (_state == ApState.Calibrate && _cal != null)
                sb.Append("Calibrating = ").Append(_cal.Progress()).Append('\n');
            sb.Append("GyroTorque_Used = ").Append(_ship.MaxTorque.ToString("0", Inv))
              .Append(_cfgGyroTorque > 0.0 ? " (pinned)" : (_ship.TorqueCalibrated ? " (measured)" : " (seed)")).Append('\n');
            if (_fuel.HasH2)
                sb.Append("Fuel_H2 = ").Append((_fuel.H2Frac * 100.0).ToString("0.0", Inv)).Append("%\n");
            if (_fuel.HasBattery)
                sb.Append("Fuel_Power = ").Append((_fuel.PowerFrac * 100.0).ToString("0.0", Inv)).Append("%\n");
            sb.Append("Fuel_Margin = ").Append(_fuelMargin).Append('\n');
            sb.Append("Perf_Instr = ").Append(_lastInstr.ToString(Inv))
              .Append('/').Append(Runtime.MaxInstructionCount.ToString(Inv))
              .Append(" peak ").Append((_peakInstrFrac * 100.0).ToString("0.0", Inv)).Append("%\n");
            sb.Append("Perf_RunMs = ").Append(_lastRunMs.ToString("0.000", Inv))
              .Append(" peak ").Append(_peakRunMs.ToString("0.000", Inv)).Append('\n');
        }

        // Runs EVERY run: peaks are the whole point and a spike between two echo rebuilds must not be
        // missed. Two comparisons and no allocation.
        void TrackPerf()
        {
            _lastInstr = Runtime.CurrentInstructionCount;
            double frac = Runtime.MaxInstructionCount > 0
                ? (double)_lastInstr / Runtime.MaxInstructionCount : 0.0;
            if (frac > _peakInstrFrac) _peakInstrFrac = frac;
            _lastRunMs = Runtime.LastRunTimeMs;
            if (_lastRunMs > _peakRunMs) { _peakRunMs = _lastRunMs; _peakTag = _prevTag; }
        }

        // Fraction of the straight-line drive detection range retained at a given cant. From
        // Spectrum's DirectionalEmission lobe: modifier is gain^2 inside the 3 deg cone and falls as
        // sec^2(base - theta) to unity at acos(1/gain) + 3 deg. Range goes as sqrt(modifier).
        static double TackRangeMult(double deg)
        {
            const double gain = 4.0, lobeDeg = 3.0;
            double baseDeg = Math.Acos(1.0 / gain) * 180.0 / Math.PI + lobeDeg;
            double m = deg <= lobeDeg ? gain
                     : deg >= baseDeg ? 1.0
                     : 1.0 / Math.Cos((baseDeg - deg) * Math.PI / 180.0);
            return m / gain;
        }

        // Rough INDEX of how much sky the cant lights relative to a fixed pencil, not a measured
        // integral. A cone at `deg` sweeps a band +/-18.5 deg wide (where the range multiplier is
        // still 2x); bias narrows the dwell toward one azimuth, so the index is faded linearly to 1.
        static double TackSweptSolid(double deg, double bias)
        {
            const double halfDeg = 18.52, rad = Math.PI / 180.0;
            double pencil = 1.0 - Math.Cos(halfDeg * rad);
            if (pencil < 1e-9) return 1.0;
            double lo = Math.Abs(deg - halfDeg) * rad;
            double hi = Math.Min(180.0, deg + halfDeg) * rad;
            double band = (Math.Cos(lo) - Math.Cos(hi)) / pencil;
            if (band < 1.0) band = 1.0;
            return 1.0 + (band - 1.0) * (1.0 - MathHelper.Clamp(bias, 0.0, 1.0));
        }

        string BuildEcho()
        {
            double frac = Runtime.MaxInstructionCount > 0
                ? (double)_lastInstr / Runtime.MaxInstructionCount : 0.0;
            StringBuilder e = new StringBuilder();
            e.Append("FlipAndBurn PB\n");
            e.Append("state: ").Append(StateName(_state));
            if (_fault.Length > 0) e.Append(" (").Append(_fault).Append(')');
            e.Append('\n');
            if (_queue != null)
            {
                if (_queue.Planning)
                {
                    e.Append("PLANNING ROUTE  ").Append(_queue.PlanWorkLeft.ToString(Inv))
                     .Append(" samples left\n");
                }
                else
                {
                    SubMission sub = _queue.CurrentSub;
                    e.Append("leg ").Append((_queue.CurrentSubMission + 1).ToString(Inv))
                     .Append('/').Append(_queue.SubMissionCount.ToString(Inv));
                    if (sub != null) e.Append("  ").Append(sub.PhaseLabel());
                    e.Append('\n');
                    QtrtController q = sub == null ? null : sub.Qtrt;
                    if (q != null && sub.Fine == null)
                        e.Append("sched ").Append(q.DbgVSched.ToString("0", Inv))
                         .Append(" along ").Append(q.AlongTrackSpeed.ToString("0", Inv))
                         .Append(" cross ").Append(q.DbgCross.ToString("0", Inv)).Append("m\n");
                }
            }
            if (QtrtController.TackAngleRad > 1e-6)
                e.Append("TACK ").Append(_tackDeg.ToString("0", Inv))
                 .Append(" deg bias ").Append(QtrtController.TackBias.ToString("0.00", Inv))
                 .Append("  (-")
                 .Append((100.0 - 100.0 * TackRangeMult(_tackDeg)).ToString("0", Inv))
                 .Append("% lit range, +")
                 .Append((100.0 / QtrtController.TackCos - 100.0).ToString("0", Inv))
                 .Append("% burn, ")
                 .Append(TackSweptSolid(_tackDeg, QtrtController.TackBias).ToString("0.0", Inv))
                 .Append("x sky)\n");
            if (_state == ApState.Calibrate && _cal != null)
                e.Append("calibrating: ").Append(_cal.Progress()).Append('\n');
            if (_align.Mode != RotationMode.None)
                e.Append("align: ").Append(ModeName(_align.Mode)).Append(' ')
                 .Append(_align.AngleDeg.ToString("0.0", Inv)).Append(" deg")
                 .Append(_align.HasDirection ? "" : " (no ref)").Append('\n');
            e.Append("v: ").Append(_ship.Body.LinearVelocity.Length().ToString("0.0", Inv))
             .Append(" m/s (phys ")
             .Append(_ship.Body.PhysicalLinearVelocity.Length().ToString("0.0", Inv)).Append(')')
             .Append(_ship.UsingModVelocity ? " [mod]" : " [derived]").Append('\n');
            if (_hasTarget)
                e.Append("dist: ")
                 .Append((_targetCoord - _ship.Body.Position).Length().ToString("0", Inv))
                 .Append(" m\n");
            if (_queue != null && !_queue.Planning)
            {
                double eta = _queue.EstimatedSecondsRemaining;
                e.Append("eta: ").Append(eta.ToString("0", Inv)).Append("s sim");
                // The plan is in sim seconds; at reduced sim speed the wall-clock wait is longer.
                if (_simSpeed < 0.98)
                    e.Append("  ").Append(RealSeconds(eta).ToString("0", Inv)).Append("s real");
                e.Append("  (sim ").Append(_simSpeed.ToString("0.00", Inv)).Append(")\n");
            }
            e.Append("thr/gyro: ").Append(_ship.ThrusterCount.ToString(Inv))
             .Append('/').Append(_ship.GyroCount.ToString(Inv))
             .Append("  wr ").Append(_ship.LastWriteCount.ToString(Inv)).Append('\n');

            // ---- fuel ----
            if (_fuel.HasH2 || _fuel.HasBattery)
            {
                e.Append("fuel:");
                if (_fuel.HasH2) e.Append(" H2 ").Append((_fuel.H2Frac * 100.0).ToString("0", Inv)).Append('%');
                if (_fuel.HasBattery) e.Append(" pwr ").Append((_fuel.PowerFrac * 100.0).ToString("0", Inv)).Append('%');
                if (_fuel.HasReactor) e.Append(" +rtr");
                e.Append('\n');
            }
            e.Append("margin: ").Append(_fuelMargin).Append('\n');
            if (_fuelMargin.StartsWith("INSUFFICIENT")) e.Append("!! FUEL SHORT FOR THIS LEG\n");
            else if (_fuelMargin.StartsWith("MARGINAL")) e.Append("!  fuel margin thin\n");

            // ---- performance ----
            // Peak is what matters: a single overrun terminates the script. The tag names what the
            // peak run was DOING, which is the difference between a one-off and a steady cost.
            e.Append("perf: ").Append(_lastInstr.ToString(Inv))
             .Append('/').Append(Runtime.MaxInstructionCount.ToString(Inv))
             .Append(" (").Append((frac * 100.0).ToString("0.0", Inv)).Append("%)")
             .Append(" peak ").Append((_peakInstrFrac * 100.0).ToString("0.0", Inv)).Append("%\n");
            e.Append("      ").Append(_lastRunMs.ToString("0.000", Inv)).Append("ms")
             .Append(" peak ").Append(_peakRunMs.ToString("0.000", Inv)).Append("ms");
            if (_peakTag.Length > 0) e.Append(" [").Append(_peakTag).Append(']');
            e.Append("  Update1\n");
            if (_probeText.Length > 0) e.Append(_probeText).Append('\n');
            if (_peakInstrFrac > 0.80) e.Append("!! INSTR >80%\n");
            else if (_peakInstrFrac > 0.50) e.Append("!  instr >50% peak\n");
            // SE logs a profiler warning around 0.5 ms/run and will flag the block as heavy.
            if (_peakRunMs > 0.5) e.Append("!  runtime >0.5ms peak\n");
            return e.ToString();
        }

        // ---------- Cost probe ----------
        //
        // Runtime.LastRunTimeMs is a whole-run figure and a PB has no Stopwatch, so the only way to
        // price one subsystem from inside the script is to run it N EXTRA times and read the slope.
        // The extra passes are idempotent -- they re-push the same commands and re-read the same
        // blocks -- so the ship never notices. Runs alternate loaded/unloaded and the minimum of each
        // is kept, which rejects scheduler noise.
        //
        //   probe thrust 200   ->   thrust 3.10us/call over 57 blocks
        //
        // Targets: thrust, gyro, refresh, ctrl, echo. Any other word stops the probe.
        string _probeWhat = "";
        int _probeN;
        int _probeRunsLeft;
        double _probeBase = double.MaxValue, _probeLoad = double.MaxValue;
        string _probeText = "";

        void StartProbe(string what, int n)
        {
            _probeWhat = what;
            _probeN = n < 1 ? 1 : n;
            _probeRunsLeft = 40;
            _probeBase = double.MaxValue;
            _probeLoad = double.MaxValue;
            _probeText = "probe " + what + " running";
        }

        void RunProbe()
        {
            if (_probeRunsLeft <= 0) return;
            // The PREVIOUS run's duration is what LastRunTimeMs reports, so bank it against whichever
            // arm the previous run was.
            double ms = Runtime.LastRunTimeMs;
            bool prevLoaded = (_probeRunsLeft & 1) == 0;
            if (prevLoaded) { if (ms < _probeLoad) _probeLoad = ms; }
            else if (ms < _probeBase) _probeBase = ms;
            _probeRunsLeft--;

            if ((_probeRunsLeft & 1) == 1)
            {
                for (int r = 0; r < _probeN; r++)
                {
                    // Only genuinely side-effect-free work belongs here. RefreshState is NOT: calling
                    // it twice in a run differences position against itself and zeroes the derived
                    // velocity. Nor is _queue.Update, which latches control state.
                    switch (_probeWhat)
                    {
                        case "thrust": _ship.ProbeThrust(); break;
                        case "gyro": _ship.ProbeGyros(); break;
                        case "reads": _ship.ProbeReads(); break;
                        case "echo": BuildEcho(); break;
                        default: _probeRunsLeft = 0; break;
                    }
                }
            }

            if (_probeRunsLeft > 0) return;
            if (_probeBase < double.MaxValue && _probeLoad < double.MaxValue)
            {
                double perCall = (_probeLoad - _probeBase) * 1000.0 / _probeN;
                _probeText = "probe " + _probeWhat + ": "
                           + perCall.ToString("0.00", Inv) + "us/call"
                           + " (base " + _probeBase.ToString("0.000", Inv)
                           + " load " + _probeLoad.ToString("0.000", Inv) + "ms x"
                           + _probeN.ToString(Inv) + ")";
            }
            else _probeText = "probe " + _probeWhat + ": no samples";
            _probeWhat = "";
            _echoCountdown = 0;
        }

        // ---------- Parsing helpers ----------

        static string Fmt(Vector3D v)
        {
            return "GPS:FB:" + v.X.ToString("0.##", Inv)
                 + ":" + v.Y.ToString("0.##", Inv)
                 + ":" + v.Z.ToString("0.##", Inv) + ":";
        }

        static string ExtractGpsName(string s)
        {
            if (s == null) return "";
            string t = s.Trim();
            if (!t.StartsWith("GPS:", StringComparison.OrdinalIgnoreCase)) return "";
            string[] p = t.Split(':');
            return p.Length > 1 ? p[1] : "";
        }

        // GPS name when there is one, else the raw argument -- so "500km" reads back as itself.
        static string DestinationName(string rest)
        {
            string n = ExtractGpsName(rest);
            return n.Length > 0 ? n : rest.Trim();
        }

        // A destination is either a coordinate or a bare distance meaning "that far straight ahead".
        // Ahead is the CONTROLLER's forward -- where the pilot is looking -- not IShip.Forward, which
        // is the max-thrust axis and can be up/down on a lander. Falls back to the drive axis only
        // when no controller is live. Suffixes: m (default) and km.
        // Peel an optional trailing "0..1" bias off a tack argument, leaving the destination in `rest`.
        // Splits on the LAST space and only commits if what remains still parses as a destination, so
        // a GPS string with spaces in its name is never chopped. Returns 0 (symmetric) when absent.
        double StripTackBias(ref string rest)
        {
            int sp = rest.LastIndexOf(' ');
            if (sp <= 0) return _tackBias;
            double b;
            if (!TryParseD(rest.Substring(sp + 1).Trim(), out b) || b < 0.0 || b > 1.0) return _tackBias;
            string head = rest.Substring(0, sp).Trim();
            Vector3D probe;
            if (head.Length == 0 || !TryParseDestination(head, out probe)) return _tackBias;
            rest = head;
            return b;
        }

        bool TryParseDestination(string s, out Vector3D v)
        {
            v = Vector3D.Zero;
            if (string.IsNullOrEmpty(s)) return false;
            string t = s.Trim();

            double scale = 1.0;
            string num = t;
            if (t.EndsWith("km", StringComparison.OrdinalIgnoreCase)) { scale = 1e3; num = t.Substring(0, t.Length - 2); }
            else if (t.EndsWith("m", StringComparison.OrdinalIgnoreCase)) { num = t.Substring(0, t.Length - 1); }

            double dist;
            if (TryParseD(num.Trim(), out dist))
            {
                dist *= scale;
                if (dist <= 0.0 || _ship == null) return false;
                IMyShipController ctrl = _ship.Controller;
                Vector3D fwd = ctrl != null ? ctrl.WorldMatrix.Forward : _ship.Forward;
                if (fwd.LengthSquared() < 1e-12) return false;
                v = _ship.Body.Position + Vector3D.Normalize(fwd) * dist;
                return true;
            }

            return TryParseVector(t, out v);
        }

        // Accepts "GPS:name:x:y:z:", "x,y,z", "x:y:z" and "x y z".
        static bool TryParseVector(string s, out Vector3D v)
        {
            v = Vector3D.Zero;
            if (string.IsNullOrEmpty(s)) return false;
            string t = s.Trim();

            if (t.StartsWith("GPS:", StringComparison.OrdinalIgnoreCase))
            {
                string[] p = t.Split(':');
                // GPS : name : x : y : z : [colour]
                if (p.Length < 5) return false;
                return TryTriple(p[2], p[3], p[4], out v);
            }

            string[] parts = t.Split(new[] { ',', ':', ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length < 3) return false;
            return TryTriple(parts[0], parts[1], parts[2], out v);
        }

        // Route syntax: waypoints separated by ';'. Each is "[kind[@tol]] <gps|x,y,z>", where kind is
        // stop | spline | fly. Unmarked waypoints are Spline, except the last, which is a Stop -- so a
        // plain list of GPS points flies through them and halts at the end. An explicit @tol (metres)
        // is the corner tolerance the speed cap is sized from; an undeclared Spline gets the mod's
        // SoftSplineTol proxy instead.
        static bool TryParseRoute(string text, out WaypointEntry[] wps, out string err)
        {
            wps = null;
            err = "";
            string[] items = text.Split(';');
            var list = new List<WaypointEntry>();
            var kinds = new List<WaypointKind>();
            var tols = new List<double>();
            var explicitKind = new List<bool>();

            for (int i = 0; i < items.Length; i++)
            {
                string item = items[i].Trim();
                if (item.Length == 0) continue;

                WaypointKind kind = WaypointKind.Spline;
                double tol = double.MaxValue;
                bool sawKind = false;

                // A leading kind token is separated from the coordinate by whitespace.
                int sp = item.IndexOf(' ');
                if (sp > 0)
                {
                    string head = item.Substring(0, sp).Trim();
                    string tolPart = "";
                    int at = head.IndexOf('@');
                    if (at >= 0) { tolPart = head.Substring(at + 1); head = head.Substring(0, at); }
                    string h = head.ToLowerInvariant();
                    if (h == "stop") { kind = WaypointKind.Stop; sawKind = true; }
                    else if (h == "spline") { kind = WaypointKind.Spline; sawKind = true; }
                    else if (h == "fly" || h == "flythrough") { kind = WaypointKind.Flythrough; sawKind = true; }
                    if (sawKind)
                    {
                        if (tolPart.Length > 0)
                        {
                            double t;
                            if (!TryParseD(tolPart, out t) || t < 0.0)
                            {
                                err = "bad tolerance '" + tolPart + "'";
                                return false;
                            }
                            tol = t;
                        }
                        item = item.Substring(sp + 1).Trim();
                    }
                }

                Vector3D c;
                if (!TryParseVector(item, out c)) { err = "bad coordinate '" + item + "'"; return false; }
                list.Add(new WaypointEntry(c, kind, tol));
                kinds.Add(kind);
                tols.Add(tol);
                explicitKind.Add(sawKind);
            }

            if (list.Count == 0) { err = "no waypoints"; return false; }

            // Unmarked final waypoint is a Stop; unmarked interior stays Spline.
            int last = list.Count - 1;
            if (!explicitKind[last])
                list[last] = WaypointEntry.Stop(list[last].Position);
            // A Flythrough with no declared tolerance takes the mod's 10 m default.
            for (int i = 0; i < list.Count; i++)
            {
                if (kinds[i] == WaypointKind.Flythrough && double.IsInfinity(tols[i]) == false
                    && tols[i] == double.MaxValue)
                    list[i] = WaypointEntry.Flythrough(list[i].Position);
            }

            wps = list.ToArray();
            return true;
        }

        static bool TryTriple(string sx, string sy, string sz, out Vector3D v)
        {
            v = Vector3D.Zero;
            double x, y, z;
            if (!TryParseD(sx, out x)) return false;
            if (!TryParseD(sy, out y)) return false;
            if (!TryParseD(sz, out z)) return false;
            v = new Vector3D(x, y, z);
            return true;
        }
    }
}

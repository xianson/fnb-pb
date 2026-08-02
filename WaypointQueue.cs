using System;
using System.Collections.Generic;
using VRageMath;

namespace IngameScript
{
    partial class Program
    {
        // Queue-entry kinds, mixable in any order.
        public enum WaypointKind
        {
            // Arrive at v = 0. Splits the queue into sub-missions.
            Stop,
            // Soft curve influence; large tolerance, capped by leg geometry during planning.
            Spline,
            // Must-visit, small tolerance.
            Flythrough,
        }

        public struct WaypointEntry
        {
            public readonly Vector3D Position;
            public readonly WaypointKind Kind;
            public readonly double Tolerance;

            // Per-waypoint chord-cut fraction for an undeclared Spline. < 0 = use the global.
            public const double UseGlobalCutFraction = -1.0;
            public readonly double CutFraction;

            public WaypointEntry(Vector3D position, WaypointKind kind, double tolerance)
                : this(position, kind, tolerance, UseGlobalCutFraction) { }

            public WaypointEntry(Vector3D position, WaypointKind kind, double tolerance, double cutFraction)
            {
                Position = position;
                Kind = kind;
                Tolerance = tolerance;
                CutFraction = cutFraction;
            }

            public static WaypointEntry Stop(Vector3D p)
            {
                return new WaypointEntry(p, WaypointKind.Stop, 0.0);
            }
            public static WaypointEntry Spline(Vector3D p)
            {
                return new WaypointEntry(p, WaypointKind.Spline, double.MaxValue);
            }
            public static WaypointEntry Spline(Vector3D p, double tolerance)
            {
                return new WaypointEntry(p, WaypointKind.Spline, tolerance);
            }
            public static WaypointEntry Flythrough(Vector3D p)
            {
                return new WaypointEntry(p, WaypointKind.Flythrough, 10.0);
            }
            public static WaypointEntry Flythrough(Vector3D p, double tolerance)
            {
                return new WaypointEntry(p, WaypointKind.Flythrough, tolerance);
            }
        }

        // Per-waypoint shrink: each interior waypoint moves toward the chord between its neighbours,
        // up to its tolerance.
        public static class WaypointShrink
        {
            public static Vector3D[] ShrinkWaypointsPerWaypoint(
                Vector3D[] waypoints, double[] tolerances, int iterations)
            {
                if (waypoints.Length <= 2) return waypoints;

                Vector3D[] original = waypoints;
                Vector3D[] result = new Vector3D[waypoints.Length];
                for (int i = 0; i < waypoints.Length; i++) result[i] = waypoints[i];
                Vector3D[] next = new Vector3D[result.Length];

                for (int iter = 0; iter < iterations; iter++)
                {
                    next[0] = original[0];
                    next[next.Length - 1] = original[original.Length - 1];
                    for (int i = 1; i < result.Length - 1; i++)
                    {
                        double tol = tolerances[i];
                        if (tol <= 0.0) { next[i] = original[i]; continue; }

                        Vector3D a = result[i - 1];
                        Vector3D b = result[i + 1];
                        Vector3D chord = b - a;
                        double chordLen = chord.Length();
                        if (chordLen < 1e-6) { next[i] = original[i]; continue; }
                        Vector3D chordDir = chord / chordLen;

                        Vector3D proj = a + Vector3D.Dot(original[i] - a, chordDir) * chordDir;
                        Vector3D offset = original[i] - proj;
                        double d = offset.Length();
                        next[i] = d <= tol ? proj : original[i] - (tol / d) * offset;
                    }
                    Vector3D[] tmp = result; result = next; next = tmp;
                }
                return result;
            }
        }

        // Splits a WaypointEntry queue into sub-missions at every Stop, plans each as a Catmull-Rom
        // spline flown by QtrtController, and runs them in order.
        //
        // Ported from the mod's Routing/WaypointQueueController.cs. The HUD route-ribbon sampling and
        // the per-sub CALIB ETA surface are not ported (display-only). MissionValidator is reduced to
        // the advisory checks a PB can act on.
        public sealed class WaypointQueueController
        {
            public readonly IShip Ship;
            public readonly OrientationController Attitude;
            public readonly WaypointEntry[] Entries;

            readonly SubMission[] _subs;
            int _idx;

            // m -- ship at first Stop within this (mod: MissionValidator.StartPosEps).
            public const double StartPosEps = 1.0;

            public bool IsDone { get; private set; }
            public int CurrentSubMission { get { return _idx; } }
            public int SubMissionCount { get { return _subs.Length; } }
            public SubMission CurrentSub { get { return _subs.Length > 0 && _idx < _subs.Length ? _subs[_idx] : null; } }

            public WaypointQueueController(IShip ship, OrientationController attitude,
                WaypointEntry[] entries)
            {
                if (entries == null || entries.Length == 0)
                    throw new ArgumentException("queue must contain at least one waypoint");

                // Prepend an implicit Stop at the ship unless the first entry already is one.
                Vector3D shipPos = ship.Body.Position;
                bool needsImplicitStart =
                    entries[0].Kind != WaypointKind.Stop
                    || (entries[0].Position - shipPos).Length() > StartPosEps;
                WaypointEntry[] effective;
                if (needsImplicitStart)
                {
                    effective = new WaypointEntry[entries.Length + 1];
                    effective[0] = WaypointEntry.Stop(shipPos);
                    Array.Copy(entries, 0, effective, 1, entries.Length);
                }
                else effective = entries;

                // U-turn promotion: an interior WP with anti-parallel in/out legs is a 180 cusp the
                // spline cannot round, so make it a real Stop.
                for (int i = 1; i < effective.Length - 1; i++)
                {
                    if (effective[i].Kind == WaypointKind.Stop) continue;
                    Vector3D dirIn = effective[i].Position - effective[i - 1].Position;
                    Vector3D dirOut = effective[i + 1].Position - effective[i].Position;
                    if (dirIn.LengthSquared() < 1e-12 || dirOut.LengthSquared() < 1e-12) continue;
                    dirIn = Vector3D.Normalize(dirIn);
                    dirOut = Vector3D.Normalize(dirOut);
                    if (Vector3D.Dot(dirIn, dirOut) < -0.99)
                        effective[i] = WaypointEntry.Stop(effective[i].Position);
                }
                Ship = ship;
                Attitude = attitude;
                Entries = effective;

                // Trivial queue (<2 entries): no path.
                if (effective.Length < 2)
                {
                    WaypointEntry restEntry = WaypointEntry.Stop(shipPos);
                    WaypointEntry[] degenSpan = new WaypointEntry[] { restEntry, restEntry };
                    _subs = new SubMission[] { new SubMission(ship, attitude, degenSpan) };
                    return;
                }

                // Slice into subs, each Stop-to-Stop (inclusive).
                List<SubMission> subs = new List<SubMission>();
                int segStart = 0;
                for (int i = 1; i < effective.Length; i++)
                {
                    if (effective[i].Kind != WaypointKind.Stop) continue;
                    WaypointEntry[] span = new WaypointEntry[i - segStart + 1];
                    Array.Copy(effective, segStart, span, 0, span.Length);
                    subs.Add(new SubMission(ship, attitude, span));
                    segStart = i;
                }
                // Trailing non-Stop sub (terminal Flythrough / Spline).
                if (segStart < effective.Length - 1)
                {
                    int spanLen = effective.Length - segStart;
                    WaypointEntry[] span = new WaypointEntry[spanLen];
                    Array.Copy(effective, segStart, span, 0, spanLen);
                    subs.Add(new SubMission(ship, attitude, span));
                }
                _subs = subs.ToArray();
                if (_subs.Length == 0) IsDone = true;
            }

            // Spline samples baked per run. The whole-route bake is ~0.7 ms at the 64-waypoint cap,
            // which no PB can afford in one run; chunking holds any single run near a tenth of that.
            // Small routes still finish inside the engage run, so nothing about them changes.
            public static int BakeSamplesPerRun = 600;

            int _bakeIdx;
            public bool Planning { get { return _bakeIdx < _subs.Length; } }

            // Bake everything now, in one call. The offline A/B harness uses this to reproduce the
            // old single-run bake and diff it against the chunked one.
            public void FinishPlanning()
            {
                while (_bakeIdx < _subs.Length)
                {
                    _subs[_bakeIdx].BakeStep(0);
                    if (!_subs[_bakeIdx].BakeDone) break;
                    _bakeIdx++;
                }
            }

            // Sample-equivalent work still owed, for the status readout.
            public int PlanWorkLeft
            {
                get
                {
                    int n = 0;
                    for (int i = _bakeIdx; i < _subs.Length; i++) n += _subs[i].BakeWorkLeft;
                    return n;
                }
            }

            public void Update(double dt)
            {
                if (IsDone || _subs.Length == 0) return;

                if (_bakeIdx < _subs.Length)
                {
                    int budget = BakeSamplesPerRun;
                    while (_bakeIdx < _subs.Length && budget > 0)
                    {
                        budget -= _subs[_bakeIdx].BakeStep(budget);
                        if (_subs[_bakeIdx].BakeDone) _bakeIdx++;
                    }
                    if (_bakeIdx < _subs.Length)
                    {
                        // PLANNING: hands off, coast. This delays the START of a long route by a few
                        // runs; it does not change how the route is then flown.
                        Ship.ThrottleForward = 0.0;
                        Ship.StrafeRight = 0.0;
                        Ship.StrafeUp = 0.0;
                        Ship.Pitch = 0.0;
                        Ship.Yaw = 0.0;
                        Ship.Roll = 0.0;
                        return;
                    }
                }

                _subs[_idx].Update(dt);
                if (_subs[_idx].IsDone)
                {
                    if (_idx >= _subs.Length - 1) IsDone = true;
                    else
                    {
                        _idx++;
                        JustAdvancedSubMission = true;
                        // Carry the committed brake latch across the boundary, as the mod's host does.
                        SubMission oldSub = _subs[_idx - 1];
                        SubMission newSub = _subs[_idx];
                        if (oldSub.Qtrt != null && oldSub.Qtrt.TermFlipDone
                            && newSub.Qtrt != null)
                            newSub.Qtrt.RestoreBrakeLatch(oldSub.Qtrt.BrakeLatchSpeed);
                    }
                }
            }

            // One-shot, set the tick _idx advances; host reads and clears it.
            public bool JustAdvancedSubMission { get; set; }

            public SubMission GetSubMission(int i) { return _subs[i]; }

            // Seconds until the whole queue completes (status readout only; approximate -- see
            // QtrtController.EstimateSecondsRemaining).
            public double EstimatedSecondsRemaining
            {
                get
                {
                    if (IsDone || Planning) return 0.0;
                    double aMax = Ship.MaxForwardThrust * Ship.Body.InvMass;
                    double total = _subs[_idx].EstimateSecondsRemaining(aMax);
                    for (int i = _idx + 1; i < _subs.Length; i++)
                        total += _subs[i].EstimateTotalSeconds(aMax);
                    return total;
                }
            }
        }

        // A single Stop-to-Stop segment.
        public sealed class SubMission
        {
            readonly IShip _ship;
            readonly OrientationController _attitude;
            readonly Vector3D _endPos;
            readonly bool _trivial;              // span had <2 distinct CPs after dedupe

            FineTranslationController _fine;
            readonly QtrtController _qtrt;
            public QtrtController Qtrt { get { return _qtrt; } }
            // Non-null only once the sub has handed the last metres to the closer.
            public FineTranslationController Fine { get { return _fine; } }

            bool _trivialDone;
            const double TrivialRestSpeed = 0.5;
            const double TrivialTimeoutSeconds = 300.0;
            double _trivialElapsed;

            // Spline corner-cut aggressiveness.
            public static double SplinePerpCutFraction = 0.0;
            // Fine-handoff overhead billed to the ETA.
            public static double FineHandoffSeconds = 2.0;
            public bool IsDone { get { return _trivial ? _trivialDone : (_fine != null && _fine.IsDone); } }

            public SubMission(IShip ship, OrientationController attitude, WaypointEntry[] span)
            {
                _ship = ship;
                _attitude = attitude;
                _endPos = span[span.Length - 1].Position;

                double totalLen = 0.0;
                for (int i = 1; i < span.Length; i++)
                    totalLen += (span[i].Position - span[i - 1].Position).Length();

                double aMax = ship.MaxForwardThrust * ship.Body.InvMass;
                double aLat = ship.MaxLateralThrust * ship.Body.InvMass;
                double vTarget = Math.Sqrt(aMax * totalLen * 0.5);

                double perpCutFraction = SplinePerpCutFraction;

                Vector3D[] positions = new Vector3D[span.Length];
                double[] tolerances = new double[span.Length];
                for (int i = 0; i < span.Length; i++)
                {
                    positions[i] = span[i].Position;
                    // Endpoints hard-pinned.
                    if (i == 0 || i == span.Length - 1) { tolerances[i] = 0.0; continue; }

                    double tol = span[i].Tolerance;
                    // A Flythrough is a HARD gate: never fillet it toward the chord.
                    if (span[i].Kind == WaypointKind.Flythrough) { tolerances[i] = 0.0; continue; }
                    // SPEND THE TOLERANCE BUDGET ONCE: a declared Spline tolerance is spent on the
                    // corner speed cap, not also on moving the control point.
                    if (span[i].Kind == WaypointKind.Spline && tol <= 1e6) { tolerances[i] = 0.0; continue; }
                    if (span[i].Kind == WaypointKind.Spline && tol > 1e6)
                    {
                        Vector3D a = span[i - 1].Position;
                        Vector3D b = span[i + 1].Position;
                        Vector3D chord = b - a;
                        double chordLen = chord.Length();
                        if (chordLen > 1e-6)
                        {
                            Vector3D chordDir = chord / chordLen;
                            Vector3D proj = a + Vector3D.Dot(positions[i] - a, chordDir) * chordDir;
                            double perp = (positions[i] - proj).Length();
                            double frac = span[i].CutFraction >= 0.0 ? span[i].CutFraction : perpCutFraction;
                            tol = frac * perp;
                        }
                    }
                    tolerances[i] = tol;
                }

                // ForSpeed fillet shrink, clamped by per-entry tolerance.
                double rTarget = aLat > 1e-9 ? (vTarget * vTarget) / aLat : 0.0;
                for (int i = 1; i < positions.Length - 1; i++)
                {
                    if (tolerances[i] <= 0.0) continue;
                    Vector3D dirIn = positions[i] - positions[i - 1];
                    Vector3D dirOut = positions[i + 1] - positions[i];
                    if (dirIn.LengthSquared() < 1e-12 || dirOut.LengthSquared() < 1e-12) continue;
                    dirIn = Vector3D.Normalize(dirIn);
                    dirOut = Vector3D.Normalize(dirOut);
                    double cosAlpha = Vector3D.Dot(dirIn, dirOut);
                    double cosHalf = Math.Sqrt(Math.Max(0.0, (1.0 + cosAlpha) * 0.5));
                    double fillet = cosHalf < 1e-4
                        ? tolerances[i]
                        : rTarget * (1.0 - cosHalf) / cosHalf;
                    tolerances[i] = Math.Min(tolerances[i], fillet);
                }
                Vector3D[] shrunk = WaypointShrink.ShrinkWaypointsPerWaypoint(positions, tolerances, 3);

                CatmullRomSpline spline;
                try
                {
                    spline = new CatmullRomSpline(shrunk);
                }
                catch (ArgumentException)
                {
                    _trivial = true;
                    return;
                }

                // Last entry's Kind drives arrival.
                QtrtController.TerminalKind qat;
                switch (span[span.Length - 1].Kind)
                {
                    case WaypointKind.Flythrough: qat = QtrtController.TerminalKind.Flythrough; break;
                    case WaypointKind.Spline: qat = QtrtController.TerminalKind.Spline; break;
                    default: qat = QtrtController.TerminalKind.Stop; break;
                }

                // SHARP-CORNER feasibility: hand each INTERIOR Spline corner's position, turn and
                // tolerance to the controller so it can cap the corner speed.
                int nC = span.Length - 2;
                Vector3D[] cornerPos = null; double[] cornerTurn = null; double[] cornerTol = null;
                if (nC > 0)
                {
                    var cPos = new List<Vector3D>(nC);
                    var cTurn = new List<double>(nC);
                    var cTol = new List<double>(nC);
                    for (int i = 1; i < span.Length - 1; i++)
                    {
                        if (span[i].Kind != WaypointKind.Spline) continue;
                        Vector3D dIn = span[i].Position - span[i - 1].Position;
                        Vector3D dOut = span[i + 1].Position - span[i].Position;
                        if (dIn.LengthSquared() < 1e-12 || dOut.LengthSquared() < 1e-12) continue;
                        double legIn = dIn.Length();
                        dIn = dIn / legIn;
                        dOut = Vector3D.Normalize(dOut);
                        double cosA = Vector3D.Dot(dIn, dOut);
                        if (cosA > 1.0) cosA = 1.0; else if (cosA < -1.0) cosA = -1.0;
                        double turn = Math.Acos(cosA);
                        // GENUINE-CORNER gate: only a real heading change gets a cap.
                        if (turn < QtrtController.CornerMinTurn) continue;
                        double tol = span[i].Tolerance;
                        if (tol > 1e6)
                        {
                            // Undeclared soft spline: a small fraction of the incoming leg, clamped.
                            double baseTol = QtrtController.SoftSplineTolLegFrac * legIn;
                            if (baseTol < QtrtController.SoftSplineTolMin) baseTol = QtrtController.SoftSplineTolMin;
                            if (baseTol > QtrtController.SoftSplineTolMax) baseTol = QtrtController.SoftSplineTolMax;
                            double frac = span[i].CutFraction >= 0.0 ? span[i].CutFraction : SplinePerpCutFraction;
                            double chordTol = 0.0;
                            if (frac > 0.0)
                            {
                                Vector3D ca = span[i - 1].Position;
                                Vector3D cb = span[i + 1].Position;
                                Vector3D cChord = cb - ca;
                                double cLen = cChord.Length();
                                if (cLen > 1e-6)
                                {
                                    Vector3D cDir = cChord / cLen;
                                    Vector3D cProj = ca + Vector3D.Dot(span[i].Position - ca, cDir) * cDir;
                                    chordTol = frac * (span[i].Position - cProj).Length();
                                }
                            }
                            tol = baseTol > chordTol ? baseTol : chordTol;
                        }
                        cPos.Add(span[i].Position);
                        cTurn.Add(turn);
                        cTol.Add(tol);
                    }
                    cornerPos = cPos.ToArray();
                    cornerTurn = cTurn.ToArray();
                    cornerTol = cTol.ToArray();
                }

                _qtrt = new QtrtController(ship, attitude, spline, qat,
                                           cornerPos, cornerTurn, cornerTol);
                _bakeLen = totalLen;
                _bakeAMax = aMax;
            }

            // Span geometry kept for the timeout widening, which cannot run until the schedule exists.
            readonly double _bakeLen, _bakeAMax;

            public bool BakeDone { get { return _trivial || _qtrt == null || _qtrt.BakeDone; } }
            public int BakeWorkLeft
            {
                get { return _trivial || _qtrt == null ? 0 : _qtrt.BakeSamplesLeft; }
            }

            // Advance this sub's schedule build; returns the sample-equivalent work consumed.
            public int BakeStep(int budget)
            {
                if (_trivial || _qtrt == null) return 0;
                bool wasDone = _qtrt.BakeDone;
                int used = _qtrt.BakeStep(budget);
                if (!wasDone && _qtrt.BakeDone)
                {
                    // The default 300 s Timeout is too short for long cruises; raise it from the schedule.
                    double estimate = _qtrt.EstimateTotalSeconds(_bakeAMax);
                    double bang = 2.0 * Math.Sqrt(_bakeLen / _bakeAMax) + 60.0;
                    if (estimate > 1e-3) bang = Math.Max(bang, estimate * 1.5 + 60.0);
                    _qtrt.Timeout = Math.Max(_qtrt.Timeout, bang);
                }
                return used;
            }

            public void Update(double dt)
            {
                if (_trivial)
                {
                    // No path; brake residual velocity with dampeners.
                    _ship.ThrottleForward = 0.0;
                    _ship.StrafeRight = 0.0;
                    _ship.StrafeUp = 0.0;
                    _ship.Pitch = 0.0;
                    _ship.Yaw = 0.0;
                    _ship.Roll = 0.0;
                    _ship.DampenersWanted = true;
                    _trivialElapsed += dt;
                    if (_ship.Body.LinearVelocity.Length() < TrivialRestSpeed
                        || _trivialElapsed > TrivialTimeoutSeconds)
                        _trivialDone = true;
                    return;
                }
                if (_fine != null)
                {
                    _fine.Update(dt);
                    return;
                }
                _qtrt.Update(dt);
                if (_qtrt.IsDone)
                    _fine = new FineTranslationController(_ship, _attitude, _endPos);
            }

            // Seconds for this sub from a v=0 start.
            public double EstimateTotalSeconds(double aMax)
            {
                if (_trivial) return FineHandoffSeconds;
                return _qtrt.EstimateTotalSeconds(aMax) + FineHandoffSeconds;
            }

            public double EstimateSecondsRemaining(double aMax)
            {
                if (IsDone) return 0.0;
                if (_fine != null) return _fine.EstimateSecondsRemaining();
                if (_trivial) return FineHandoffSeconds;
                return _qtrt.EstimateSecondsRemaining(aMax) + FineHandoffSeconds;
            }

            // Phase label for the status surface.
            public string PhaseLabel()
            {
                if (_trivial) return "rest";
                if (_fine != null) return "fine:" + FineTranslationController.PhaseName(_fine.CurrentPhase);
                return "qtrt:" + _qtrt.DbgPhase;
            }
        }
    }
}

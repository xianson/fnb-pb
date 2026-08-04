# FlipAndBurn — Programmable Block port

A Space Engineers PB script that flies a ship from where it is to a GPS point and arrives
**stopped**, using the flip-and-burn profile (accelerate → flip → brake → fine terminal closer),
plus standalone attitude alignment modes.

The control laws are ported from the FlipAndBurn mod
(`D:\fnb-merge\mods\FlipAndBurn\Data\Scripts\FlipAndBurn\WaypointQueue\`). They are not
reimplementations — the maths is transcribed. See "What was ported" below.

## Install

1. Build (see "Building"), or copy the packed script by hand.
   The build deploys to `%APPDATA%\SpaceEngineers\IngameScripts\local\FlipAndBurnPb\script.cs`.
2. In game: place a Programmable Block, edit it, **Browse Scripts → Local → FlipAndBurnPb**, OK.
3. The ship needs a ship controller (cockpit, flight seat, or remote control), thrusters, and gyros.
4. On first run the block writes its own CustomData with the control fields filled in.

## Commanding it

Two equivalent surfaces: **run arguments** (one-shot) and **CustomData fields** (persistent).
A PB cannot create terminal controls, so CustomData is the analogue of the mod's `FB_*` /
`AutoRotate_*` terminal properties, and the field names are kept recognisably close.

### Arguments

| Argument | Effect |
|---|---|
| `goto GPS:Dest:12345.6:-789:4321:` | Set target and fly to it, arriving stopped |
| `goto 12345.6,-789,4321` | Same, plain coordinates |
| `goto` | Fly to the target already in `FB_TargetCoord` |
| `goto 400km` | Fly 400 km along the **cockpit's** forward — point and go |
| `tack 400km` | Same flight, drive held off the track to dim the Spectrum signature (see below) |
| `tack 400km 0.6` | Tack with an explicit asymmetry bias, `0`–`1` |
| `route <wp>;<wp>;...` | Fly a multi-waypoint route (see below) |
| `target GPS:...` | Set the target without engaging |
| `align prograde` | Nose along velocity |
| `align retrograde` | Nose against velocity |
| `align gravity` | Nose away from the gravity source ("up") |
| `align gps` | Nose at `FB_TargetCoord` |
| `align target` | Same as `align gps` (see limitations) |
| `align none` | Release alignment |
| `stop` | Disengage everything and release all overrides |
| `calibrate` | Measure gyro torque and rate cap (**spins the ship**, ~20 s) |
| `rescan` | Rebuild the block cache and reset |
| `resetperf` | Clear the sticky instruction/runtime peaks and the peak tag |
| `probe <what> <n>` | Price one subsystem in µs/call (`thrust`, `gyro`, `reads`, `echo`) |

`align` also accepts the short forms `pro`, `retro`, `grav`, `tgt`, `off`, and the mod's numeric
`RotationMode` values (`0`–`4`, `6`).

### Routes

Waypoints are separated by `;`. Each is `[kind[@tol]] <gps|x,y,z>`:

| Kind | Meaning |
|---|---|
| `spline` | Soft curve influence — the path bends toward it. **Default for unmarked waypoints** |
| `fly` | Must-visit. Hard cross-track gate, never filleted toward the chord (10 m default tolerance) |
| `stop` | Arrive at v = 0. **Splits the route into sub-missions.** Default for the *last* waypoint |

```
route GPS:A:10000:0:0: ; GPS:B:20000:5000:0: ; GPS:C:30000:0:0:
```

flies a smooth curve through A and B and stops at C. `@tol` sets the corner tolerance in metres,
which is what the corner speed cap is sized from — `spline@500 GPS:B:...` allows a wider, faster
corner. An undeclared `spline` gets the mod's `SoftSplineTol` proxy (1% of the incoming leg, clamped
to 25–75 m) instead.

Each `stop` starts a fresh sub-mission with its own spline and speed schedule, so
`route ... ; stop GPS:Mid:... ; ...` is how you force a full halt part-way.

### Tack — flying without a searchlight on

`tack` flies **exactly the route `goto` would**. The only difference is that the drive is held at
an angle to the track, so the engine's emission lobe stops pointing down it.

**Why this matters.** Spectrum models a drive as a *directional* emitter: all fifteen SDX drive
variants are `AngleDegrees 3`, `Gain 4`, `ScaleWithThrust`, on the Optical band. Inside that 3°
cone the detection range is **4× the spherical value**; it decays as `sec²` and reaches unity at
`acos(1/4) + 3° = 78.5°`. The lobe is keyed on the block's cardinal hull face, so **it rides your
attitude, not your velocity**.

That makes a plain flip-and-burn the worst possible profile. It aims a 4× searchlight at your
**origin** for the whole acceleration, then flips and aims it at your **destination** for the
whole brake — and you are closer to the destination when it does. `tack` exists to fix that.

Range retained, for a 5×5 civilian drive at full throttle (69 km spherical / 276 km in-lobe):

| Cant | Range mult | Propellant | Retained |
|---:|---:|---:|---:|
| 0° (`goto`) | 4.00 | — | 100% |
| 10° | 2.73 | +2% | 68% |
| 20° | 1.92 | +6% | 48% |
| **30°** (default) | **1.51** | **+15%** | **38%** |
| 45° | 1.20 | +41% | 30% |
| ≥78.5° | 1.00 | +∞ | 25% |

30° is the knee — past ~45° the lobe is already flat and the propellant curve is not. Throttling
down only buys `sqrt`, so **pointing beats throttling**.

#### The bias argument — an exposure trade, not just efficiency

The optional `0`–`1` float controls what the cant azimuth does over time.

- **`0` (default, symmetric).** The azimuth cones round at a constant rate. Lateral velocity is
  mean-zero by construction, so it costs almost nothing in trajectory — but the sweeping lobe
  lights roughly **6× the solid angle** a fixed pencil does. It protects whoever is on your track
  by broadcasting to an annulus a straight burn would never have touched. Each bearing in that
  band is lit ~21% of the period, so integrated exposure is only ~1.3× worse — but the number of
  *distinct* bearings that learn you exist is 6× higher, and one logged sample is enough.
- **`1` (fixed azimuth).** The lobe stays a pencil, so total exposure is the same as a straight
  burn while the on-track observer is still protected. You pay for it in continuous lateral thrust
  that the cross-track loop fights, which bows the flown path.

That bow is the point: a symmetric tack is transparent to anyone who **integrates** your track
rather than sampling it — the mean bearing is still true. Bias is the only knob that produces a
genuinely deceptive path.

Echo reports all of it: `TACK 30 deg bias 0.60 (-62% lit range, +15% burn, 3.1x sky)`.

#### Caveats

- **The cant fades out at the terminal.** The brake gate zeroes the drive at 15° of nose error,
  and the cone's precession spends part of that budget on tracking lag; a brake that gates its own
  throttle off does not recover. So the cant ramps to zero once the hard brake or terminal arrest
  latches. **Your final approach is un-tacked** — the lobe does point at the destination when you
  are closest to it. Partial protection, deliberately.
- **Untested in flight.** Watch `AlignErr` during the brake on a sluggish hull; raise `TackCone`
  if it climbs toward 15°. `TackAngle = 0` disables the feature entirely.
- What the cross-track loop settles to when fighting a biased tack is gain-dependent and has not
  been measured. That is where the "efficiency lost" actually lands.
- A PB cannot read Spectrum (`RegisterMessageHandler` is not whitelisted), so the script has no
  idea where anyone is. `tack` protects the two bearings it can infer — origin and destination —
  and nothing else.

### CustomData

Everything above the status marker is input; everything below it is written by the script.

```
FB_TargetCoord = GPS:Dest:12345.6:-789:4321:
AutoRotate_Mode = None
FB_Engage = false
Controller =
GyroRateCap = 1

; ---- status (written by the script) ----
Autopilot_State = Idle
...
```

| Field | Meaning |
|---|---|
| `FB_TargetCoord` | Destination. Accepts `GPS:name:x:y:z:`, `x,y,z`, `x:y:z`, `x y z` |
| `AutoRotate_Mode` | `None`/`Prograde`/`Retrograde`/`Gravity`/`GPS`/`Target`. Edge-triggered |
| `FB_Engage` | `true` starts the A→B run, `false` aborts it. Edge-triggered |
| `Controller` | Substring of the ship controller to use as reference. Blank = auto |
| `GyroRateCap` | Gyro max angular rate, rad/s. Written by `calibrate` |
| `GyroTorque` | Total gyro torque, N·m, or `auto` to measure. Written by `calibrate` |
| `ArrivalDist` | Optional arrival tolerance override, metres (default 40) |
| `FB_Route` | A route string, same syntax as the `route` argument. Edge-triggered |
| `SplineSamples` | Schedule samples per spline segment. **48 = mod-identical**; lowering it changes the flown path (see Performance) |
| `TackAngle` | Cant used by `tack`, degrees. Default 30, capped 60. `0` disables |
| `TackCone` | Azimuth period, seconds. Default 60. Raise it if `AlignErr` climbs during the brake |
| `TackBias` | Default bias when `tack` is issued without one. `0`–`1`, default 0 |

Status fields written back: `Autopilot_State`, `Fault`, `AutoRotate_Aligned`, `AutoRotate_Angle`,
`Speed`, `PhysicalSpeed`, `Autopilot_RemainingDistance`, `Autopilot_TargetName`, `Leg`, `Phase`,
`Sched_Speed`, `Along_Speed`, `CrossTrack`, `AlignErr`, `StraightRun`, `BrakeCommitted`,
`Autopilot_ETA`, `ArrivalError`, `GyroTorque_Used`, `Fuel_H2`, `Fuel_Power`, `Fuel_Margin`,
`Perf_Instr`, `Perf_RunMs`.

`Leg` is `n/total` sub-missions. `Phase` is `qtrt:<accel|track|brake|hover>` while the spline
follower owns the ship, then `fine:<Burn|Flip|Brake|Translate|...>` for the terminal close.

Editing CustomData takes effect on the next run; the script detects the change by comparing against
what it last wrote, so its own status writes do not re-trigger parsing.

## Gyro calibration — run this once per ship

Two numbers the mod reads from the world are **not on the PB whitelist**:

- **Gyro torque.** The mod calls `Grid.GetTotalGyroTorque()`, which sums
  `MyGyroDefinition.ForceMagnitude * GyroPower`. `MyGyroDefinition` is not reachable from a PB.
- **Gyro rate cap.** The mod reads
  `MyDefinitionManager.Static.EnvironmentDefinition.LargeShipMaxAngularSpeedInRadians`. Also
  unreachable.

Hardcoding the vanilla constants (3.36e7 / 4.48e5 N·m, and a 1.0 rad/s fallback) is wrong the
moment anyone installs a gyro mod or changes the world's angular speed limit — and both numbers
feed the flip-time estimate, which is what decides when to start the flip and therefore where the
ship stops. So they are **measured** instead.

Run `calibrate` once per ship (or after refitting gyros):

```
calibrate
```

It drives each body axis to saturation for ~2.5 s, reverses to null the rate, and settles —
roughly 20 s total. **It deliberately spins the ship**, so do it clear of obstacles. Dampeners are
held on throughout, and thrusters stay released, so the ship rotates in place rather than drifting.
It refuses to start while an A→B run is engaged.

It measures two things from the response:

- **Torque** = `peak(|dω/dt|) × I` about each axis, largest of the three kept. SE clamps gyro
  output to a sphere so authority is isotropic, and each per-axis figure is a *lower* bound (the
  override servo ramps in over up to 120 frames, then coasts at the rate cap where α ≈ 0), so the
  maximum is the right estimator.
- **Rate cap** = the ω the ship plateaus at while still commanding full authority. Only accepted
  if the rate visibly *stopped* accelerating (α fell below 5% of peak while ω was high) — otherwise
  the spin simply ran out of time and the reading would be meaningless, so it is discarded and the
  existing value kept.

**Both results are written back into the block's CustomData** (`GyroTorque`, `GyroRateCap`), so
they survive recompiles, are visible, and can be overridden by hand.

Without calibration the script still runs: it falls back to opportunistic passive measurement
during flight (same `torque = α·I` estimator, sampled whenever it happens to be commanding ≥50%
authority), seeded from the vanilla constants until the first real sample arrives. Status shows
which is in use — `GyroTorque_Used = ... (pinned | measured | seed)`. Calibrating is strongly
preferred, because passive measurement only ever sees the authority the mission demanded.

## The velocity problem — read this

The FlipAndBurn mod's `SpeedController` gives every grid a **HighSpeed cruise** that reaches
thousands of m/s. While it is engaged, SE's *physical* velocity is clamped near 95–100 m/s and the
real motion is applied by teleport-stepping the grid. `IMyShipController.GetShipVelocities()`
returns the **clamped** value.

A brake schedule fed ~100 m/s while the ship is actually doing 8,000 m/s underestimates the stopping
distance by ~6400×, and the ship sails straight past the target.

So the script does **not** use `GetShipVelocities()` for control. It has two sources, in order of
preference.

### 1. `FB_Velocity` from the mod (preferred)

The mod publishes `FB_Velocity` on the Flight Computer — the *same* `IRigidBody.LinearVelocity`
its own controllers plan against (virtual in HighSpeed, SE physics otherwise). It also publishes
`FB_PhysicalVelocity`. When a Flight Computer is present the script reads both directly, which is
strictly better than any local estimate: no filter lag, no spike gate, no CoM-jump artefacts, and
it is by construction the number the ported laws were tuned against.

The Flight Computer is located by probing for the `FB_Velocity` property rather than by subtype
name, so a renamed block still works, and the read is exception-guarded so a world without the mod
degrades cleanly instead of killing the script.

### 2. Position differencing (fallback)

Without the mod, or without a Flight Computer, the script derives velocity by differencing the
ship's world-space centre of mass across runs against the actual elapsed
`Runtime.TimeSinceLastRun`:

- Centre of mass, not grid origin: it is rotation-invariant, so a spinning ship does not inject a
  false `ω × r` term.
- Actual elapsed time, never an assumed 1/60 — PB run spacing varies with load.
- Spike-gated: a per-run change larger than `(aMax + |g|)·dt·4 + 50 m/s` is rejected, because CoM
  jumps when cargo shifts or a grid docks, and HighSpeed motion is not smooth integration. After 3
  consecutive rejections the sample is accepted anyway, so a genuine discontinuity recovers.
- Smoothed with a 0.5 EMA.

Either way the mapping onto the mod's abstraction is the same: the effective value is
`IRigidBody.LinearVelocity` (what every control law consumes), and the clamped physics value is
`IRigidBody.PhysicalLinearVelocity` (which `FineTranslationController` uses only for the
conservative `gateSpeed = max(virtual, physical)` arrival gate). Both are wired to the same
consumers as in the mod.

Which source is live is reported as `Speed = ... (FB_Velocity)` or `(derived)` in CustomData, and
as `[mod]` / `[derived]` in the PB detail area. The position history is maintained even while the
mod source is in use, so losing the Flight Computer mid-flight falls back to a warm estimator.

Because the plan is solved from velocity at construction, engaging a run **defers** building the
profile until at least two position samples exist. Out of HighSpeed the derived value equals the
physical one, so nothing changes.

## Architecture

The porting seam is `IShip` / `IRigidBody`, copied from the mod. Everything above it came across
unchanged; only the adapter below it is new.

| File | Origin |
|---|---|
| `IShip.cs` | Mod's `Autopilot/Control/IShip.cs`, minus nothing |
| `MathHelpers.cs` | Verbatim |
| `OrientationController.cs` | Verbatim except `ControlPeriod` (below) |
| `FineTranslationController.cs` | Verbatim |
| `QtrtController.cs` | The spline follower. Flight law verbatim; ETA + dodge dropped (below) |
| `CatmullRomSpline.cs` | Verbatim, minus the display-only `Injected` arrays |
| `WaypointQueue.cs` | `WaypointEntry` + `WaypointShrink` + `SubMission` + `WaypointQueueController` |
| `AlignController.cs` | `GyroController.TryGetTargetDirection` + mode plumbing |
| `PbShip.cs` | New. The PB analogue of `SEShip.cs` |
| `GyroCalibrator.cs` | New. Measures what the PB whitelist hides |
| `Program.cs` | New. Entry point, CustomData surface, state machine, fuel guard |

### The flight path

Identical in shape to the mod. A queue of `WaypointEntry` is sliced into **Stop-to-Stop
sub-missions**; each sub-mission builds one centripetal Catmull-Rom spline through its span, bakes a
curvature-limited speed schedule over it, and flies it with `QtrtController` — pure-pursuit
lookahead nose, lateral PD cross-track, flip-and-burn terminal brake — handing the last ~80 m to
`FineTranslationController`.

A bare `goto X` is **not** a special case: it is a one-waypoint route. The queue prepends an
implicit `Stop` at the ship, so it becomes a single Stop-to-Stop sub-mission flown by exactly the
same law as any leg of a longer route.

### Verified against the mod

The port was A/B'd against the mod's own `QtrtController` + `WaypointQueueController` compiled from
`D:\fnb-merge` into the same test binary, both driving identical plants from identical state,
stepped in lockstep at 1/60 s. Across 3 hulls (agile / heavy / low-gyro) × 4 scenarios (50 km and
200 km straights, a 2-waypoint dogleg, a 3-waypoint S-curve):

```
max positional divergence over the whole flight: 0.000000 m
```

not merely at the endpoint — every tick. `EstimateTotalSeconds`, `Timeout` and `PeakSchedSpeed`
also match to < 1e-9 on every sub-mission. The harness lives in the session scratchpad
(`scratchpad/abtest`) and is not part of this project.

### What was dropped from `QtrtController`, and why it is safe

Two blocks did not come across. Neither is in the control path:

- **The HUD/route ETA machinery** (CALIB-B/D/F/H/I/J/K — `EtaFineGrid`, `HudSecondsFrom`,
  `EtaTimeFraction`, `EtaStopCapSpeed`, the closed-loop march). ~16 k characters of display-only
  code that computes per-waypoint route times for a HUD a PB does not have. The one function in
  that block that *does* affect flight is `EstimateTotalSeconds`, which sets the sub's `Timeout`;
  it is kept, rewritten in closed form, and verified numerically identical (see above). The
  rewrite is exact because with `stopCap=false` the mod's capped branch never runs, so `t == tPlain`
  and its `EtaFlipCharge` reduces to `+_tFlip`; `EtaScheduleFactor()` is 1.0 either way because
  `EtaCurveFactor == 1.00`.
- **The flight-time asteroid dodge.** A PB has no voxel or procedural-generator API, so the mod's
  `DodgeProbe` would be null, `UpdateDodge` would return `Vector3D.Zero` every tick, and the tracked
  reference would be the bare on-path point. The port hard-codes that outcome. **This is not
  obstacle avoidance and there is no substitute for it — see Limitations.**

`ScheduleIsCurvy` went with the ETA block because `EtaCurveFactor` is 1.00, making
`EtaScheduleFactor()` identically 1.0.

### The one deliberate change to a control law

`OrientationController`'s discrete min-time switching law has a zero-order-hold correction
(`v·h/2` in the switching function, and `h` in the `sBand` and the deadbeat terms). In the mod the
law runs every engine tick, so that `h` and the engine tick were the same number, `1/60`.

A PB holds its command for a whole PB run interval. Sampling a switching curve with a stale hold
time is precisely the defect the ZOH term exists to fix — it fires the brake late, overshoots and
limit-cycles. So `h` is now `ControlPeriod`, set from `Runtime.TimeSinceLastRun`. The same
substitution was applied to the two per-tick rate estimates (`setpointRate`, measured α).

`AccelToRateGap` still divides by 60 and still uses `rf/60.0`: that inverts SE's fixed 60 Hz gyro
servo ramp, which is engine physics, not the control period.

**`ControlPeriod ≈ 1/60` always, so this reduces to the original code exactly.** The rescaling
mechanism is retained only because the loop period is read rather than assumed.

### Gyro units

`IMyGyro.Pitch/Yaw/Roll` are **absolute rad/s**, not a `[-1,1]` fraction of gyro power. `PbShip`
matches the mod's corrected `SEShip.ApplyGyros`: the `[-1,1]` body command is multiplied by
`GyroRateCap`, rotated into world space through the reference matrix, then rotated into each
gyro's own local frame via `transpose(gyro.WorldMatrix)` — gyro axes are not the reference frame's
unless the gyro happens to be mounted aligned. The attitude law normalises by the same
`Ship.GyroRateCap`, so the command that leaves the law and the number the adapter writes agree.

### Thrust

Per-axis budgets are summed honestly from `MaxEffectiveThrust` of working thrusters, bucketed by
the dominant axis of each thruster's push direction in the reference body frame — so asymmetric
ships report six independent budgets, as the mod's `IShip` expects. The drive axis is resolved
first (the grid-local axis with the most thrust becomes "forward"), mirroring `SEShip`, because
hulls whose main drive is not along grid −Z would otherwise read `MaxForwardThrust = 0`.

Overrides are released on disengage, on arrival, and on `stop`. Note that
`ThrustOverridePercentage = 0` *disables* the override — that is what lets SE's dampeners finish
the stop when the closer asks for them.

## Fuel guesstimate

The mod has a real fuel guard that reserves brake authority. This is a rougher thing, but it is
**measured rather than assumed**: a PB cannot read a thruster's consumption rate, so instead of
hardcoding one the script watches how fast the stores actually drain against how many
newton-seconds it commanded, and divides.

- Stores: hydrogen tanks (`Capacity × FilledRatio`) and batteries (`CurrentStoredPower`). A working
  reactor is treated as removing the *electrical* limit, since uranium draw is negligible next to
  thruster fuel.
- Impulse is integrated from **commanded** thrust and the per-sign budgets, not from
  `IMyThrust.CurrentThrust` — that would mean walking every thruster every run.
- Windows with negligible impulse are discarded (a tiny fuel delta over a tiny impulse is all
  noise, and a recharging battery makes the delta negative).

Reported as:

```
fuel: H2 62% pwr 88% +rtr
margin: OK (410s have / 190s need)
```

`need` is `FineTranslationController.RequiredBurnSeconds()` — a read-only addition that returns the
*powered* seconds the current plan still wants (accel to hop peak plus the retro brake). Coast and
flip are unpowered, so it is well under the ETA. The margin is `OK` ≥1.5×, `MARGINAL` ≥1.0×,
`INSUFFICIENT` below.

**It is advisory and does not abort a run**, deliberately: cutting thrust mid-brake strands the
ship worse than arriving dry. Treat `INSUFFICIENT` as "do not engage", not as a safety net. It also
needs a burn to calibrate against, so it reads `measuring` until the first real thrust window.

## Performance

Reported every run in the PB detail area and in CustomData:

```
perf: 3184/50000 (6.4%) peak 11.2%
      0.081ms peak 0.190ms  Update1
```

with escalating warnings — `!  instr >50% peak`, `!! INSTR >80%`, and
`!  runtime >0.5ms peak` (SE flags a block as heavy around there). Peaks are sticky, because a
single overrun terminates the script and an averaged number would hide it; clear them with
`resetperf` after a fix. `Runtime.CurrentInstructionCount` is sampled inside the echo block, so it
excludes the few hundred ops of the echo itself.

`peak` carries a **tag** naming what the peak run was doing — `[firstrun]`, `[engage]`, `[plan]`,
`[rescan]`, `[status]`, `[fuelscan]`, `[arg]`, `[tick]`. This matters more than the number: a spike
tagged `firstrun` is JIT and will never recur, one tagged `tick` is a steady cost. `LastRunTimeMs`
read inside `Main` is the *previous* run's duration, so the tag is credited accordingly.

**The instruction counter is not a proxy for wall-clock.** SE counts operations executed by the
*script*; a call into the game costs wall-clock and almost nothing on the counter. A run reading
1.6% of the instruction budget can still take 0.75 ms if it makes a few hundred ModAPI calls. When
`perf:` shows a low instruction count and a high millisecond figure, the cost is block I/O, and no
amount of tuning the control maths will touch it.

The per-run control loop is deliberately cheap:

- Block lists are scanned every 300 runs, not per run.
- Each thruster's axis bucket is computed **once at scan time** and cached, because a thruster's
  orientation relative to the grid is fixed. The per-run write is a table lookup, not a transform.
- Mass, inertia, thrust budgets and gyro torque refresh every 30 runs (0.5 s at Update1) — the same
  cadence as the mod's `SEShip.CacheRefreshTicks`.
- **Overrides are written only when they change.** Every thruster and gyro caches the exact value
  last pushed to it; an identical write is skipped, because the block retains what it was given. The
  comparison is exact — an epsilon would round the command, which would be a behaviour change. One
  block per run is re-written unconditionally, so an override moved by the pilot or another script
  self-corrects within a second. The `wr` figure on the `thr/gyro:` line is how many writes the last
  run actually issued; in steady cruise it should sit near 1, not near `thrusters + 4 × gyros`.
- `IsWorking` / `IsFunctional` / `Enabled` are sampled with the thrust budgets rather than per run,
  so the write gate and `MaxForwardThrust` agree on which blocks are live.
- Fuel stores are read every 30 runs, the block lists behind them every 600, and a reactor's
  running state with the stores rather than twice per run.
- `Me.CustomData` is polled every 10 runs and only written when the text actually differs from what
  the block already holds.
- The Echo panel is rebuilt every 10 runs and re-emitted from cache in between; formatting ~20
  doubles at 60 Hz is wasted work for a readout nobody can read that fast.

If `peak` is still high with a `tick` tag, use `probe` to find out where it goes rather than
guessing. `probe thrust 200` runs `ApplyThrust` 200 extra times with the write cache dropped, and
reports µs per call from the slope of `LastRunTimeMs` — the only way to price one subsystem from
inside a PB, which has no `Stopwatch`. The extra passes re-push the same values, so the ship is
unaffected. Targets are `thrust`, `gyro`, `reads` (the per-run ModAPI reads) and `echo`; anything
else that is not side-effect-free is deliberately not offered.

### Cost of the spline follower

Measured offline against the same binary the A/B harness flies (net48, so the runtime matches SE's;
expect in-game figures ~1.3-1.5x higher for SE's instruction-counter injection):

| route | worst single run at engage | steady tick | planning runs |
|---|---|---|---|
| 1 waypoint (`goto`) | 0.013 ms | 3.5 µs | 1 |
| 3 waypoints | 0.037 ms | 6.3 µs | 1 |
| 12 waypoints | 0.147 ms | 20.0 µs | 1 |
| 24 waypoints | 0.197 ms | 36.9 µs | 2 |
| 48 waypoints | 0.291 ms | 70.5 µs | 4 |
| 64 waypoints | 0.290 ms | 91.4 µs | 6 |

The old point-to-point path measured ~0.7 µs/tick.

Two distinct costs:

- **Per tick**, dominated by `RuntimeBrakeCeiling`, which walks every remaining schedule sample with
  an `Acos` and a `Sqrt` each. Samples per sub-mission are `48 x segments + 1`. This is flight
  logic — it is the fix for the spline-knot overshoot — so it is not trimmable. What *was* trimmed
  is the rate cap it re-derived from `IShip` on every one of those samples; hoisting it is
  arithmetically inert and took the 12-waypoint tick from 34.8 to 20.0 µs.
- **At engage**, the schedule bake. Three things happened to it:
  - `CatmullRomSpline.Sample` evaluates the segment **once** for position, tangent and curvature.
    `Position` and `CurvatureAt` derived the same `(seg, t)` and each called `EvalSegment`.
  - The knot vector is **cached per segment**. It is a pure function of `seg` but cost three
    `Math.Pow` per *sample*; 141 of every 144 of those calls were recomputing an unchanged value.
    This also speeds up every per-tick `ClosestU`, whose 8 iterations usually stay in one segment.
  - The corner windows are **binary-searched**. `_sSamp` is monotone, so a window is a contiguous
    index range; the four full scans per corner were the bake's only O(waypoints squared) term.
    The bounds are padded and the caller still applies the original predicate, so the hit set is
    identical rather than merely equivalent.

  Together those took the 64-waypoint bake from ~2.6 ms to 0.71 ms, bit-identically.

- **The bake is then chunked across runs** (`WaypointQueueController.BakeSamplesPerRun`, 600). A
  route of 12 waypoints or fewer still bakes inside the engage run, exactly as before. Longer ones
  spend a few runs in a **PLANNING ROUTE** state — hands off, coasting — before the first burn. This
  changes *when* a long route starts moving, by up to a tenth of a second; it does not change how it
  is then flown. Everything the bake reads from the ship is frozen at engage (thrust and lateral
  budgets, attitude bandwidth, rate cap, gravity, and the hot-start position/velocity seed), so the
  schedule cannot depend on which run a chunk happened to land in. The A/B harness proves this: a
  chunked bake at 7, 97 and 600 samples per run produces byte-identical schedules and trajectories
  to the single-run bake, across routes from 1 to 64 waypoints.

One lever remains, and it is the only knob here that changes the flown path:

- `SplineSamples` (default **48** = mod-identical). Lowering it coarsens the speed schedule
  linearly in both costs.

If a long route overruns, prefer splitting it into shorter `route` commands over turning that knob.
The loop always runs at 60 Hz; there is no rate throttle.

## Limitations — what is NOT ported

Be aware of these before trusting it:

- **No obstacle avoidance of any kind.** This is the big one. `AsteroidRouter`, `PlanetRouter`,
  `ScannerDodgeProbe` and Qtrt's flight-time `ObstacleProbe` dodge are all absent, and **no
  substitute was written**. A PB cannot read voxels or query the procedural generator, so there is
  nothing to build one from. It will fly straight through whatever is in the way. The mod's
  `SpeedController` still does its own HighSpeed collision checks, but the *route* is unguarded.
  Do not read the ported dodge fields' absence as "avoidance is handled" — it is not handled at all.
- **No `PlanetRouter`.** A leg that would pass through a planet is not re-routed around it. The
  gravity-descent hover-hold and the gravity-baked speed schedule *are* ported, so a landing on a
  leg that already points at the pad works; planning a leg that clears terrain does not exist.
- **No `MissionValidator`.** The mod refuses or warns on degenerate queues (duplicate points,
  U-turn-ish corners, tolerance collapse, singular inertia). Only the U-turn *promotion* (an
  anti-parallel interior waypoint becomes a real `Stop`) and the implicit-start `Stop` are ported,
  because those change the flown path. A malformed route is rejected only if the spline constructor
  throws, which surfaces as `Fault = route rejected: ...`.
- **The fuel guard is approximated.** The staging is the mod's — latched, escalate-only, coast at
  1.15× then force-brake at 1.02× — and it drives the same `Qtrt.FuelCoast` / `Qtrt.FuelForceBrake`
  hooks. But the mod compares *fuel units* from `GetFuelRemaining`/`GetMaxFuelConsumption`, which a
  PB cannot call; both sides are instead expressed as seconds of full main-drive burn from
  `FuelEstimator`. Same shape, coarser input. The mod's `Abort("fuel_dry")` is not ported.
- **`SportMode` returns false and `BrakeDragDecel` returns 0.** Both are `SpeedController` state
  with no PB-readable equivalent. Consequence: braking is *not* drag-aware, so an in-atmosphere
  stop will under-brake relative to the mod. Sport mode's aggressive policy is simply unavailable.
- **`GravityAt(worldPos)` returns the ship's own gravity** rather than gravity at an arbitrary
  point (`MyAPIGateway.Physics.CalculateNaturalGravityAt` is not whitelisted). Qtrt's schedule bake
  *does* consume this — `_gSamp[i] = GravityAt(p)` per sample. In vacuum both are zero and the
  schedule is identical. Near a planet the bake sees one uniform field instead of one that rotates
  along the path, so a long leg through a gravity well gets a less accurate accel/brake budget than
  the mod's. The A/B above ran in vacuum, where this term is exactly zero.
- **Alignment roll is not controlled.** The mod's `GyroController.TryGetTargetUp` locks roll to
  radial-out from the nearest planet via `MyGamePruningStructure`, which is not whitelisted.
  `OrientationController` only ever aims the nose and nulls off-axis rate — it has no roll setpoint —
  so this matches what the ported law can express, but it is a real difference from the mod's
  standalone alignment.
- **`RotationMode.Target` aliases GPS.** The original reads the WeaponCore AI focus target through
  `Session.WcApi`; a PB has no access to WeaponCore. As instructed, `Target` points at
  `FB_TargetCoord`.
- **Inertia is a box approximation of the main grid only** (`Max - Min` cells × `GridSize`), like
  the mod. Subgrid mass is included in `Mass` (via `CalculateShipMass().PhysicalMass`) but not in
  the inertia box, so a ship with large rotor-mounted subgrids will have its flip time
  underestimated.
- **Construct scope is `IsSameConstructAs`**, which covers rotor/piston subgrids. The mod uses
  `GridLinkTypeEnum.Physical`, which also spans connector-docked grids. Docked ships are therefore
  outside this script's actuator set.
- Sub-metre arrival is not claimed. The default `ArrivalDistM` is the mod's 40 m. The mod's
  validated 0.1 m result came from a `Holding` routing stage in `WaypointQueueComponent` that is
  not ported.
- No re-engagement after `Arrived`. It stops, enables dampeners, releases overrides and goes idle.
- A run in progress does not survive a recompile — the queue, its splines and its baked schedules
  are not persisted in `Storage` (target and mode are). Re-issue the `route`/`goto`.
- **The mod's HUD/route ETA is not ported**, so `Autopilot_ETA` is the plain schedule integral from
  the current position, not the mod's closed-loop CALIB march. It reads optimistic during a terminal
  brake. Nothing in the flight path consumes it.

## Building

```
dotnet build "D:\fnb-pb\FlipAndBurnPb.csproj" -c Release -v:minimal --nologo
```

**Use `dotnet build`, not VS2022's MSBuild.exe.** Both compile, but `Mal.Mdk2.PbAnalyzers` 2.1.17
is built against Roslyn 4.12 while VS2022 17.10's MSBuild ships Roslyn 4.10, which refuses to load
it (`warning CS9057`) — the whitelist then silently is not checked. The .NET 9 SDK's Roslyn loads
it and enforces `MDK01`.

Output: `%APPDATA%\SpaceEngineers\IngameScripts\local\FlipAndBurnPb\script.cs`.

### Minification is load-bearing

`FlipAndBurnPb.mdk.ini` sets **`minify=full`**. This is not cosmetic — SE caps a PB script at
100,000 characters, and the spline follower does not fit without it:

| `minify=` | packed size | fits? |
|---|---|---|
| `stripcomments` | 155,425 | no — 55 % over |
| `lite` | 107,654 | no — 8 % over |
| `full` | **59,872** | yes, 40 % headroom |

Measured, not projected — all three levels were built. Note these are **characters**, which is what
SE limits and what MDK reports; `wc -c` overcounts by ~12 % because it charges 2 bytes per CRLF.

`full` renames identifiers, so an in-game runtime exception will name mangled types. If you need a
readable stack trace while debugging, switch to `lite` *and* temporarily drop `SplineSamples`
— but nothing else fits, and the flight algorithm was not trimmed to make the number work.

### Verifying the packed output actually compiles in game

MDK strips every `using` at pack time and SE prepends its own fixed set, so a type outside that set
compiles locally and fails in game with `CS0103`. `dotnet build` will not catch it. The gate is to
wrap the *packed* `script.cs` in SE's real editor template — only the namespaces the game injects,
`LangVersion 6`, referencing `Bin64` — and compile that. A harness for this lives in the session
scratchpad (`scratchpad/segate`); it is not part of this project. Both `System.Globalization` uses
here are fully qualified for exactly this reason, and `IFormatProvider` is genuinely prohibited by
the whitelist, so `Inv` is typed as `CultureInfo`.

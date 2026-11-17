# Velocity Axes Plugin - Plan

Status: **implemented, untested on hardware** (2026-11-16). See section 15.

## 1. Goal and usage model

Run selected non-XYZ axes as independent, velocity-controlled motors while coordinated
G-code motion (XYZ) and Direct Motion (DM) continue to run - the Mach3 "velocity jog" behaviour.

Target machine family:

- 1 or 2 solder pumps. Idle and active speeds are changed at runtime (wave height, nozzle size);
  the client steps pump speed in a closed loop while watching wave height.
- 1 or 2 conveyors. Stop / slow / fast with controlled acceleration; the conveyor advances until
  the client sees a sensor, then the client changes speed. The advance speed is set per program.
  A second-stage holding conveyor runs fully asynchronously from the rest of the system.
- Which axes are reserved is **configuration, not firmware**: an unforeseen machine layout must
  not require reflashing.

Same source on STM32F4xx and STM32H7xx; only grblHAL HAL APIs are used.

## 2. Findings

### 2.1 Existing core building blocks (both platforms)

| API | Purpose |
|---|---|
| `hal.stepper.claim_motor(axis, claim)` | Removes an axis from the main stepper ISR. Driver `stepper_step_out()` / `stepper_dir_out()` skip claimed axes, so neither the planner nor DM (which uses `hal.stepper.pulse_start`) can drive the pins. |
| `hal.stepper.output_step(step, dir)` | Outputs a step pulse (plus direction) for any axes; pulse end is handled by the driver's inject timer. Applies `$3` direction invert. |
| `hal.timer.claim/configure/start/stop` | Portable timer API; identical semantics in F4 and H7 `Src/timers.c`. |

All three require `STEP_INJECT_ENABLE=1`, which is currently off in both builds
(`grbl/driver_opts.h` only enables it for the stepper spindle or plasma plugin).

### 2.2 Why not the core `stepper2.c` (st2) motor driver

- One timer per motor at 1 us resolution; 16-bit timers overflow above ~65 ms/step, i.e. slow
  pump/conveyor speeds or low acceleration break.
- Spindle-oriented units and speed limiting.
- Two motors calling `output_step()` close together lose a pulse: the second call overwrites the
  driver's `step_pulse.inject.out`, so the first pin is never cleared.

A single shared timer that steps all reserved axes in one `output_step()` call avoids all three
and needs no driver change.

### 2.3 Command paths vs machine state

| Path | IDLE | CYCLE | JOG (DM/`$J`) | HOLD | ALARM |
|---|---|---|---|---|---|
| G-code / M-code line | yes | yes | **rejected** (`Status_SystemGClock`) | yes | **rejected** |
| `$` system command | yes | yes | yes | yes | yes |
| `$nnn=` setting write | yes | no | no | no | yes |

Hence a `$` command is needed for control that must always work (section 4.2).

### 2.4 Side effects of reserving an axis

- **Motor enable**: the core de-energises motors when idle (`$1`) and on reset. The plugin must
  wrap `hal.stepper.enable` and keep running reserved axes enabled (same technique as the
  stepper spindle plugin). PMC-02-AA has per-axis enable pins; boards with a single shared enable
  pin cannot power a reserved axis independently of XYZ (section 12).
- **G-code / `$J`**: the planner would still count steps for a commanded reserved axis while the
  motor does not move. Must be rejected (section 7).
- **Homing**: a reserved axis in a homing cycle would fail. Must be excluded (section 7.2).
- **DM**: would integrate `sys.position` for an axis it cannot step. Must be rejected (section 7.3).
- **Timers**: PWM aux outputs claim their timers during driver init, before plugins load, so the
  plugin claiming one generic timer at init does not steal a PWM timer.

### 2.5 Availability checks (both trees)

- Settings `$790-$799`: unused (nearest fork additions are `$780/$781`, G64).
- M-codes `M1020-M1029`: unused (fork PWM plugin uses `M1010-M1012`; core enum ends at `M960`).
- `$VEL`: unused system command name.
- Error code 200: unused (core codes end at 88; 253-255 are internal).

## 3. Configuration

### 3.1 Reservation mask - `$790`

`Format_AxisMask`. Each axis A and above (`axis index >= 3`) is independently reservable; e.g.
reserving only B leaves A and C as normal G-code axes. Bits for X, Y, Z are refused with
`Status_SettingValueOutOfRange`.

### 3.2 Per-axis options - `$791..$795`

One bitfield per reservable axis, only registered for axes that exist (`axis index < N_AXIS`):

| Setting | Axis index | grblHAL letter |
|---|---|---|
| `$791` | 3 | A |
| `$792` | 4 | B |
| `$793` | 5 | C |
| `$794` | 6 | U (client "D") |
| `$795` | 7 | V (client "E") |

Bits:

| Bit | Name | Default |
|---|---|---|
| 0 | Keep running on soft reset | off |
| 1 | Stop on feed hold / safety door | off |

Alarms are not configurable per axis; the set of alarms that stops all reserved axes is a
global setting (section 3.5).

### 3.5 Stopping alarms - `$796`

`Format_Bitfield`, one bit per alarm code (bit `n-1` = alarm `n`, labels from `alarms.h`).
Default: alarms 10, 14, 16, 17, 19, 20, 21, 22. Alarm 10 (E-stop) is always forced on.
Alarm codes without a bit (added to the core later) always stop.

Direction invert uses the existing `$3`; no plugin option.

### 3.3 Axis parameters

Reuse the normal axis settings so units stay consistent with the rest of the machine:

- `$10x` steps/unit - set to steps/rev to make speeds RPM.
- `$11x` max rate (units/min) - commanded speeds are clamped to it.
- `$12x` acceleration (units/s^2) - default ramp; can be overridden at runtime (section 4).

### 3.4 Applying changes

Settings are only writable in IDLE/ALARM. On change of `$790`:

1. Axes leaving the mask: immediate stop, `claim_motor(axis, false)`. No position resync is needed
   because the axis position was never changed while reserved.
2. Axes entering the mask: `claim_motor(axis, true)`.

The effective mask is re-evaluated whenever backend capability changes (e.g. when the PMC slave
bridge binds U/V to DM's remote backend, those axes are released from local reservation).

No reboot required. Reserved axis positions (`sys.position`) are frozen while reserved, so the
G-code parser and planner stay consistent when the axis is released.

## 4. Runtime commands

Presets are dropped: the client sends new speeds directly.

Axes are addressed by **index** (`P3` = A ... `P7` = V) to avoid ambiguity between grblHAL letters
(U/V) and client names (D/E).

### 4.1 `M1020` - queued set speed (G-code programs, MDI)

```
M1020 P<axis> Q<speed> [R<accel>]
M1020 P<axis> R<accel>
```

- `Q` signed units/min; sign = direction; `Q0` ramps to stop. Clamped to `$11x`.
- `R` acceleration override in units/s^2; `R0` restores `$12x`. Persists until changed or reset.
- **Queued**: takes effect in program order, after all preceding motion and before any
  following motion, without stopping the planner (section 4.5).
- Subject to the line rules in 2.3 (rejected in JOG/ALARM).

### 4.2 `$VEL` - set speed in any state

```
$VEL=P<axis> Q<speed> [R<accel>]
$VEL                                 report all reserved axes
```

Same words as `M1020`, but **immediate**: applied when received, in every state (IDLE, CYCLE,
JOG, HOLD, ALARM), regardless of queued `M1020` commands. A queued `M1020` for the same axis that
applies later overrides the `$VEL` value at that point.
**Exception**: while the E-stop input is active (`hal.control.get_state().e_stop`), a non-zero `Q`
is rejected with `error:50` (`Status_EStop`); `Q0`, `R` and reports are still accepted.
Handled by a registered system command (not the G-code parser), so state locks in 2.3 do not apply.
Intended for the client's closed-loop adjustments (wave height) that must not wait for motion.

### 4.3 `M1021` - report

```
M1021 [P<axis>]
```

Output per reserved axis: `[VEL:<axis>,Q=<target>,V=<actual>,R=<accel>,F=<flags>]`, with `,H` appended
while the axis is held by feed hold. `R` shows the effective acceleration (`$12x` if not overridden).

### 4.4 Errors

| Condition | Response |
|---|---|
| `P` missing / not an integer | `error:` `Status_GcodeValueWordMissing` / `Status_BadNumberFormat` |
| Axis not reserved | `error:200` (section 7.4) |
| `M1020` queue full (32 pending) | `error:11` (`Status_Overflow`) |
| `Q` or `R` out of range / negative `R` | `error:` `Status_GcodeValueOutOfRange` |

### 4.5 Queued execution of `M1020`

Options considered:

| | A: `user_mcode_sync` (core, no changes) | B: motion-synchronised output command (chosen) |
|---|---|---|
| Order | After all preceding motion | After all preceding motion |
| Motion | **Full stop**: planner drains, no lookahead/blending across the M-code | Continuous; applied at the block boundary |
| Stream | `ok` delayed until preceding motion completes; client stalls | `ok` immediately; client keeps streaming |
| `[EXEC:N]` | When executed | When applied |
| Core change | None | Small (below) |

B uses the same mechanism as synchronised `M62/M63/M67`: the parser attaches pending output
commands to the next planner block, and the stepper ISR applies them when that block starts
(`stepper.c`, "Execute output commands to be synchronized with motion").

Core change: `output_command_t` gains a callback variant (`exec(context)` + context pointer)
alongside digital/analog. The plugin's callback only stores target/accel, so it is ISR-safe.

Apply rule - the earlier of:

1. The start of the next motion block (normal case: `M1020` between two moves).
2. All preceding motion completed and no motion pending (`plan_get_current_block() == NULL` and
   no G64 blend candidate held, `mc_path_blend_pending_age()`). Covers `M1020` as the last line
   or followed by non-motion lines. Checked from `on_execute_realtime`.
3. Immediately at parse time if nothing is queued or moving.

All three satisfy "after preceding moves, before following ones". A per-command state flag
(pending -> applied, changed with interrupts disabled) guarantees exactly-once application when
1 and 2 race. Errors in the line (`gc_at_exit` with failure) cancel pending commands as today.

`[EXEC:N]` for the `M1020` line is reported when the speed is applied (not when the ramp ends).

## 5. Motion engine

### 5.1 Timer

One `hal.timer` claimed at init, 1 us timebase, periodic at `VELOCITY_AXES_TICK_HZ`
(build option, default 40000). Started when any reserved axis has non-zero speed, stopped when
all are idle.

### 5.2 Step generation

Per axis, a 32-bit phase accumulator: each tick `acc += increment`; carry = one step.
`increment = |v| / tick_hz * 2^32`. Any low speed works; max one step per tick per axis.
All axes stepping in a tick are emitted by a single `hal.stepper.output_step()` call.

### 5.3 Ramps

Every `tick_hz / 1000` ticks: `v += clamp(target - v, +/-accel / 1000)` in float steps/s, then
recompute `increment` and direction. Reversal ramps through zero. Commands only write
`target` and `accel` (single 32-bit stores); the ISR owns `v`.

### 5.4 Step-rate limits

- Plugin: max step rate = tick rate. Tick rate is not a hard limit; it is bounded by
  step pulse length + delay (`$0`, `$29`) < tick period, and CPU cost. Step timing jitter is up to
  one tick, so keep normal operation below ~50% of the tick rate for smooth motion.
- DM, for reference: fixed 40 kHz ISR (`DM_ISR_HZ`) and Q16 `int32` steps/s, so DM is limited
  to **32767 steps/s** and its default vmax overflows above that (section 12).
- Planner motion: limited only by the driver's step timer (well above either).

### 5.5 Motor enable

`hal.stepper.enable` is wrapped: `enable |= running reserved axes`. When an axis stops, the
last core-requested enable state is re-applied from the foreground.

## 6. Stop and safety behaviour

| Event | Behaviour |
|---|---|
| E-stop | Immediate stop, all reserved axes (alarm 10, cannot be cleared from `$796`). Non-zero speeds rejected while the input is active (4.2). |
| Stopping alarm (`$796`) | Immediate stop, all reserved axes. |
| Other alarms | Reserved axes keep running. |
| Soft reset (Ctrl-X) | Immediate stop unless option bit 0. Applies whether or not motion was active (alarm 3 itself does not stop). |
| Feed hold / safety door | Ramp to stop if option bit 1; previous target restored on cycle resume. |
| `$790` change | Affected axes stop (section 3.4). |

"Immediate" = output stops on the next tick, no ramp. Alarms are detected in the plugin ISR by
polling `sys.rt_exec_alarm` (as DM does) and checking the code against the stop list, so the stop
does not wait for the foreground. Queued `M1020` commands are discarded whenever the core discards
the motion they are attached to (reset, any alarm that flushes the planner), even if the axis
keeps running at its current speed.

Alarm sources (`grbl/alarms.h`, both cores) and the `$796` default:

| Code | Alarm | Typical cause | Stops reserved axes (default) |
|---|---|---|---|
| 1 | Hard limit | Limit switch tripped during motion | no |
| 2 | Soft limit | Target outside travel | no |
| 3 | Abort cycle | Soft reset while moving (position lost) | no (reset handled by option bit 0) |
| 4, 5 | Probe fail | Probe already triggered / no contact | no |
| 6-9, 15, 18 | Homing fail | Reset, door, pull-off, approach, auto-square, generic | no |
| 10 | E-stop | E-stop input | **yes** (forced) |
| 11 | Homing required | Startup with homing enabled | no |
| 12 | Limits engaged | Startup with a limit switch active | no |
| 13 | Probe protect | Probe triggered during non-probe motion | no |
| 14 | Spindle | Spindle at-speed/fault | **yes** |
| 16 | Self-test failed | Startup self-test | **yes** |
| 17 | Motor fault | Stepper driver fault input | **yes** |
| 19 | Modbus exception | Modbus device (VFD etc.) | **yes** |
| 20 | Expander exception | I/O expander | **yes** |
| 21 | NVS failed | Settings storage | **yes** |
| 22 | Client timeout | Client supervision - client lost | **yes** |

Alarm codes added later (> 22) always stop until given a bit in `$796`.

## 7. Exclusion of reserved axes

### 7.1 G-code and `$J`

Chained `grbl.on_motion_dispatch` (fork hook, present in both cores) rejects at the validate
stage any request whose `axes` mask contains a reserved axis. Covers G0/G1/G2/G3, G28/G30 stored
positions, canned cycles and `$J` jogs. Rejected before any motion is queued.

### 7.2 Homing

Chained `grbl.home_machine` strips reserved axes from each cycle; an empty cycle is skipped.
An explicit single-axis home of a reserved axis (e.g. `$HB`) returns `error:200`.

### 7.3 Direct Motion

`direct_motion.c` checks the reserved mask in velocity, absolute-move and relative-move paths
before touching DM state. Rejected axes are reported per section 7.4; other axes in the same
masked velocity frame are still applied.

### 7.4 Error reporting

| Source | Response | Reason given? |
|---|---|---|
| G-code / `$J` line | `error:200` | Code is unique; `$ERRORS`/`$EA` lists "Axis is reserved for velocity control" (registered with `errors_register`). With `GCODE_ERROR_DETAILS_ENABLE=1` (H7 build has it; F4 does not) the core currently appends a generic "External motion validation failed" - section 11 changes this to name the axis. |
| `$HB` etc. | `error:200` | As above. |
| DM frame | `[DM:ERR,<seq>,<type>,6,<axis>]` | New `DM_Err_AxisReserved = 6`, with the axis index as detail (today's generic reject is `4`). |
| `M1020`/`$VEL` on unreserved axis | `error:200` | As above. |

## 8. Status reporting

Reuse `|AV:` (no new field, no conditional parsing). `st_get_realtime_axis_rates()` in both cores'
`stepper.c` gets one hook at its end that overwrites reserved-axis entries with the plugin's actual
ramped speed (signed units/min). Applies whether the rest of `|AV:` came from DM or the planner,
and in all states (a pump running while IDLE reports its speed).

`|MPos`/`|WPos` for reserved axes stay frozen (section 3.4).

## 9. Backend architecture and remote (slave CPU) axes

### 9.1 Split

The plugin is split into a **front end** and **backends** so the PMC slave can be added later
without touching command parsing, safety, exclusion or reporting.

| Front end (one, generic) | Backend (per axis) |
|---|---|
| Settings (`$790-$795`), reservation mask | Produces motion for the axes it owns |
| `M1020` / `$VEL` / `M1021` parsing, queueing (4.5) | Ramps to target with given accel |
| Safety events (section 6) | Immediate stop |
| G-code/DM/homing exclusion (section 7) | Reports actual speed |
| `|AV:` overlay (section 8) | Declares which axes it can serve |

### 9.2 Backend interface (`velocity_axes.h`)

```c
typedef struct {
    axes_signals_t axes;                                         // axes this backend can drive
    bool  (*set_target)(uint8_t axis, float units_per_min, float accel_units_s2); // ISR-safe, latch only
    void  (*stop)(axes_signals_t axes, bool immediate);          // ISR-safe, latch only
    float (*get_rate)(uint8_t axis);                             // units/min, signed
    void  (*poll)(void);                                         // foreground, optional
} velocity_axes_backend_t;

bool velocity_axes_register_backend (const velocity_axes_backend_t *backend);
```

Rules:

- `set_target`/`stop` may be called from the stepper ISR (queued `M1020`, section 4.5) and from the
  plugin ISR (alarm stop), so they only latch values. Any I/O (e.g. a slave packet) happens in
  `poll()` from the foreground - the same pattern `pmc_slave_dispatch.c` already uses for DM
  (`dm_remote_velocity()` latches, `dm_poll()` submits).
- A `$790` bit is accepted only if a registered backend covers that axis. The built-in **local**
  backend (section 5) registers axes 3..`N_AXIS-1` minus any axes bound to DM's remote backend.
- Each axis is served by exactly one backend; the first registration wins.

### 9.3 PMC slave (later)

On PMC-02-AA, axis indices 6/7 (grblHAL U/V, client D/E) belong to the slave CPU through
`pmc_slave_dispatch.c`. Until a slave backend exists, `$790` refuses those bits (no backend).
The future slave backend registers axes 6/7 from `pmc_slave_dispatch_bind()` and maps:

| Front end | Slave |
|---|---|
| `set_target` | `PmcSlave_JogVelocity` (already used for DM velocity) |
| `stop` | `PmcSlave_Stop` / jog velocity 0 |
| `get_rate` | `remote->status.velocity[]` (already used by `dm_remote_state()`) |

To confirm then: slave-side acceleration control, any slave jog watchdog, and that remote
`G-code` dispatch for a reserved slave axis is rejected by section 7.1 before reaching
`dispatch_motion()`.

## 10. Direct Motion binary path (deferred)

| | `$VEL` / `M1020` | DM binary frame |
|---|---|---|
| Processing point | Line protocol | Telnet byte RX, ahead of line buffer |
| Latency | ms; can wait behind a line blocked on a full planner | Immediate |
| Transport | Any stream | Telnet only |
| Acknowledge | `ok` / `error:` | Velocity frames: none |
| Watchdog | None | 1 s heartbeat stops velocity mode - needs per-axis opt-out for pumps |
| Coupling | Plugin only | DM must route reserved axes without entering JOG |

Decision: ship section 4 first; add this only if measured latency is a problem.

## 11. File changes (both F4 and H7 unless noted)

| File | Change |
|---|---|
| `plugins/velocity_axes.c` (new) | The plugin. |
| `plugins/velocity_axes.h` (new) | `velocity_axes_reserved()`, `velocity_axes_get_rates()`. |
| `plugins/direct_motion.c` | Section 7.3, `DM_Err_AxisReserved`. |
| `grbl/driver_opts.h` | `VELOCITY_AXES_ENABLE` default 0; forces `STEP_INJECT_ENABLE 1`. |
| `grbl/plugins_init.h` | Call `velocity_axes_init()`. |
| `grbl/stepper.c` | `|AV:` hook (section 8); apply callback output commands (section 4.5). |
| `grbl/core_handlers.h`, `grbl/protocol.c` | New `grbl.on_soft_reset` event, raised for Ctrl-X before `mc_reset()`; distinguishes a soft reset from alarm-driven resets. |
| `plugins/direct_motion.h` | `direct_motion_remote_axes()` so the local backend skips remote-bound axes. |
| `grbl/gcode.h`, `grbl/gcode.c` | `output_command_t` callback variant; helper for a user M-code to queue one (section 4.5). |
| `grbl/gcode.c` | Use the dispatch handler's detail text instead of the fixed "External motion validation failed" (optional, section 7.4). |
| `platformio.ini` | `-D VELOCITY_AXES_ENABLE=1` for `pmc_02_aa_6axis`/`pmc_02_aa_8axis` and derived envs (F4; the 3-axis env has no reservable axes) and `waveshare_openh743i_c` (H7). |

No driver (`Src/`) changes.

## 12. Known limitations and related issues

- Boards with a single shared stepper enable pin cannot de-energise XYZ independently of reserved axes.
- `output_step()` sets direction and step together; with `$29 = 0` the first step after a reversal may
  violate driver DIR setup time. Reversals ramp through zero, so at most one step is affected; set
  `$29` if this matters.
- DM `default_vmax_steps_s_q16` overflows above 32767 steps/s (pre-existing, fix separately).
- Reserved axes are not position-tracked; a G-code program cannot rely on their position.

## 13. Open questions

None.

Resolved 2026-11-16: stopping alarms are a setting (`$796`, 3.5) defaulting to 10, 14, 16, 17, 19,
20, 21, 22 with E-stop forced; alarm 3 does not stop (soft reset governed by option bit 0); `M1020`
attaches to the next move (4.5); `$VEL` immediate in all states except non-zero speeds during
active E-stop; slave backend deferred with interface per 9.2; tick rate 40 kHz.

## 14. Test plan

1. Build `pmc_02_aa_8axis` and `waveshare_openh743i_c`.
2. Reserve B only: confirm A/C still accept G-code; `G1 B10` returns `error:200`; `$H` skips B.
3. `$VEL=P4 Q600` while idle, during a G-code program, during a DM jog, and in feed hold.
4. Scope: step frequency vs commanded speed at low (1 step/s) and high (20 kHz) rates; ramp time vs `$124`/`R`.
5. Two reserved axes at different rates: no lost pulses (count steps on scope/counter).
6. E-stop, each alarm class (stopping vs non-stopping per section 6), soft reset while idle and while
   moving (alarm 3), feed hold - with each option bit set/cleared.
7. `|AV:` shows reserved-axis speed in IDLE, CYCLE and JOG.
8. Idle timeout (`$1`) does not de-energise a running reserved axis.
9. `$790` change releases/claims axes; released axis position matches parser position.
10. Queued `M1020`: between two moves (applied at the boundary, no stop, `[EXEC:N]` at that point);
    as the last line of a program; after a G64-blended move; cancelled by reset/alarm.
11. `$VEL` and queued `M1020` on the same axis interleaved.
12. E-stop active: `$VEL=P4 Q100` returns `error:50`, `$VEL=P4 Q0` returns `ok`.
13. `$796` changes: clear alarm 22, simulate client timeout, reserved axes keep running; bit 10 cannot be cleared.

## 15. Implementation status

Built (2026-11-16): `pmc_02_aa`, `pmc_02_aa_8axis` (F4), `waveshare_openh743i_c` (H7). Not yet run on hardware.

Implementation notes:

- `plugins/velocity_axes.c` is identical in both trees; `M1020` = set (queued), `M1021` = report.
- Settings `$791..$795` are named "Velocity axis <index> options".
- Alarm detection: plugin ISR and foreground poll `sys.rt_exec_alarm` for new codes; `on_state_change`
  (ALARM/ESTOP) is a backstop.
- Homing: reserved axes are stripped from each cycle and from `sys.homing.mask` (so they never block
  "homing required"); `$H<letter>` for a reserved axis returns `error:200`.
- Motor enable: before the core has issued its first enable request, starting/stopping a reserved axis
  assumes all non-reserved motors are enabled.
- `|AV:` overlay is applied after both the DM and planner paths of `st_get_realtime_axis_rates()`.

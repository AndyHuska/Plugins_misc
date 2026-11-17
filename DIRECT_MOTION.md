# DirectMotion Manual/Setup Contract

DirectMotion exists for responsive keyboard jogging and concurrent independent
manual/setup positioning. Example: index a conveyor while the operator adjusts
other axes. It is not the coordinated process-motion planner; use ordinary
G-code/G64 for queued process paths. The firmware does not know the host UI's
operation category, so the client must preserve this distinction.

Local DM owns a separate 1 kHz profile controller and 40 kHz step ISR. Each axis
has its own position/velocity state; no vector path coordination is implied.
New local position commands retarget that axis rather than queue a process path.
Idle transitions reconcile normal parser/planner position with actual counters.
Core state/command restrictions still apply: this change does not authorize
arbitrary G-code or coordinate-zeroing commands while the controller is in JOG.

## Axes And Compatibility

Configured axis count is N_AXIS (currently eight on the full F4/H7 targets), not
three. Axis indices: 0 X, 1 Y, 2 Z, 3 A, 4 B, 5 C, 6 U, 7 V. Legacy machine
names D/E correspond to U/V at indices 6/7. Disabled/unconfigured indices reject.
Local step/direction masks and position accounting cover every configured axis.

The binary frame envelope, version 1 and CRC16-CCITT behavior are unchanged.
All scalar fields below are signed little-endian Q16.16. Historical names say
mm, but values follow the configured machine-axis units. Per-second DM rates
are NOT G-code F values in units/minute.

| Type | Payload | Meaning |
| --- | --- | --- |
| 0x01 | 12 bytes: X/Y/Z velocities | Legacy XYZ velocity vector only |
| 0x02 | Existing heartbeat payload | Refresh velocity watchdog |
| 0x03 | Existing stop payload | Legacy stop |
| 0x10 | 13 bytes: mask + three velocities | Legacy XYZ masked velocity; non-XYZ mask bits reject |
| 0x10 | 1 + 4*N_AXIS bytes: mask + all axis velocities | Extended masked velocity |
| 0x11 | 13 bytes: uint8 axis, target, vmax, acceleration | Independent absolute position move |
| 0x12 | 13 bytes: uint8 axis, delta, vmax, acceleration | Independent relative move from actual position |
| 0x13 | uint8 axis | Stop selected axis, subject to remote legacy limitation below |
| 0x14 | Existing empty payload | Stop all |

On an eight-axis target the extended masked payload is 33 bytes. Include all
eight velocity fields, even if only A is masked; unmasked values are ignored.
Legacy XYZ commands must not zero or update a running A-V axis. Absolute and
relative position-frame sizes never change. The decoder rejects unknown masks,
bad lengths and extended fields not represented by the supplied payload.

For A-V velocity jogging the client must send the full configured-axis masked
payload; an old XYZ-only packet cannot carry an A velocity. Keyboard key-up
should zero only the jogged axes via a masked velocity update (or their axis
stop commands), not STOP_ALL, which would also cancel an independent conveyor
position move. The existing position-move payload needs no client format change.

`$I+` reports `[DM:CAP,N=8,V=33,R=0]` on an eight-axis unbound target. N is the
axis count, V the full masked-velocity payload length, R the bound remote-axis
mask in decimal. R=192 means U/V route to an external backend. R=0 means every
configured axis remains local. Discover this report; do not infer new DM frame
support solely from the core's AXS count on an older firmware.

`[DM:OK,sequence,type]` is command acceptance, not target completion. Position
completion/activity comes from mode/phase/status reports and real position.
Remote queued requests can be rejected later with `[DM:ERR,sequence,type,4,axis]`.
The velocity watchdog does not stop independent position moves simply because
there is no key heartbeat. Separate position and velocity lifecycles in the UI.

## Optional F4 Slave Backend

`direct_motion_bind_remote()` assigns an explicit remote-axis mask. The plugin
never generates local profile steps or increments local counters for those
axes. Callbacks enqueue bounded work; physical transport and packet conversion
run in foreground, never in the step ISR. Remote status supplies actual position
and per-minute rates. Stop flags cross ISR/foreground atomically.

`pmc_slave_dispatch_bind()` installs U/V DM routing when a slave bridge is
actually bound. Absolute/relative DM requests become legacy MoveIndependent 13
with the sync completion flag, after units/limits conversion. Each slave axis
has a separate outstanding completion slot, so U and V can run concurrently
without holding the normal RPC queue until completion. Native tests cover
out-of-order completion and queries during these moves.

Legacy slave position retargeting while that same axis is moving is rejected;
do not equate this with the local controller's retargeting support. Relative
remote targets use fresh actual step telemetry, not parser queued position.
Remote velocity uses JoggingVelocityIndependent 27 with steps/sec conversion;
pending velocity updates are coalesced per axis. The peer's status/watchdog and
flow-control behavior still need physical validation.

Legacy StopMotion 28 stops both slave axes. A U or V stop therefore requests a
slave-global stop; immediate versus decelerating behavior is limited by that
API. It cannot guarantee stopping if the link is broken. A future negotiated
selected-axis stop can improve this without silently changing old semantics.

The physical slave transport is not enabled by these changes. Until it is
initialized and bound, F4 uses the existing temporary local U/V pins. Do not
claim remote hardware behavior has been tested from a successful build.

## Validation

Native tests: F4 `tests/direct_motion_protocol_test.c` links this fork's
`direct_motion_protocol.c`; `tests/pmc_slave_protocol_test.c` exercises the
slave codec, transaction/conversion layers and concurrent independent moves.
F4 eight-axis and H7 Waveshare builds pass. H7 retains its pre-existing spindle
configuration warnings. No target was flashed or physically actuated for this
change. Runtime multi-axis timing, cancellation, limits, watchdog and coexistence
with keyboard operations require a controlled bench test before release.
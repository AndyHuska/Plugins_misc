# DirectMotion Manual/Setup Contract

DirectMotion is for responsive keyboard jogging and concurrent independent
manual/setup positioning, not coordinated process paths. For example, a conveyor
can index while other axes are adjusted. Process moves belong in the normal
G-code/G64 planner. Core command/state restrictions still apply; this change
does not authorize arbitrary G-code or coordinate-zeroing commands during JOG.

All N_AXIS axes are supported locally: X,Y,Z,A,B,C,U,V at indices 0..7 on this
eight-axis target. Legacy machine D/E names correspond to U/V. Every local axis
has its own position/velocity controller; their paths are not coordinated.

The frame version/envelope/CRC are unchanged. Types 0x11/0x12 keep 13-byte
single-axis position payloads (axis plus Q16.16 target/delta, vmax and accel).
Type 0x01 stays a 12-byte XYZ velocity vector. Type 0x10 accepts the old 13-byte
XYZ masked vector or a full 33-byte mask/eight-velocity payload. Non-XYZ mask
bits in an old 13-byte packet reject, rather than reading missing values.
Unmasked axes are unchanged. Velocity/acceleration are per-second machine-axis
units; G-code F remains units/minute. Absolute DM targets are machine positions;
relative targets start from actual position, not queued parser state.

Clients must use the full masked frame for A-V velocity jogging. On keyboard
release, zero/stop only the jogged axes rather than STOP_ALL; the latter also
cancels a concurrent independent conveyor move.

`$I+` reports `[DM:CAP,N=8,V=33,R=0]`: axis count, full masked payload bytes and
remote mask. H7 has no remote backend by default; all axes remain local.
`DM:OK` means accepted, not finished. Track phase/activity and actual position
for completion. Local position targets can be retargeted without a process
queue; velocity heartbeat timeout does not cancel position moves.

The shared remote-backend interface exists for the F4 slave bridge but is
inactive on H7. Remote axes, when explicitly bound, never emit local pulses.
Detailed client/slave behavior is documented in the F4 sibling fork's
`plugins/DIRECT_MOTION.md`, including the old slave's global-stop limitation.

Native velocity-decoder tests and both F4/H7 builds pass. Runtime multi-axis
timing/safety and keyboard coexistence still require bench validation. Nothing
was flashed or actuated for this change.
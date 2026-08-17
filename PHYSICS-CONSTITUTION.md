# PHYSICS-CONSTITUTION.md — lilkuzco_kinetics

The twelve invariants, as **ratified 2026-08-16** at the v0.1.0 gate.

These are build-failing. A violation is a P0 physics bug: **fix the model, never special-case
the symptom.** Several cannot be violated by any code path at all, and that is the stronger
arrangement — there is nothing to check because there is nothing to break.

| | Enforced by | Where |
|---|---|---|
| I1 continuity | check, every substep | `Invariants.checkContinuity` + `SweptCollision` |
| I2 bounded forces | **construction** | clamp applied before the state update, no unclamped value exists |
| I3 energy honesty | check, every substep | `Invariants.checkEnergy` |
| I4 mass accounting | **construction** + check | mass derived, not stored; one fuel door |
| I5 drag/lift sanity | **construction** | drag built as a negative multiple of the airspeed unit vector |
| I6 turn authority | **construction** | turns come from lift the airframe can actually make |
| I7 determinism | golden hashes | `GoldenTests`, `java.lang.Math` only |
| I8 fuzz immunity | 10,000 cases/run | `FuzzTests` |
| I9 no magic numbers | check + loader | `Constants` throws on a missing key |
| I10 single damage door | **construction** | sealed event hierarchy, no damage field anywhere |
| I11 scale audit | generated + verified | `ScaleAudit` |
| I12 quaternion sanity | check, every substep | renormalised unconditionally |

---

## Amendments ratified at the v0.1.0 gate

Three changes to the constitution as originally written. Two came from the fuzz harness, one
from the golden battery. All three were fixed in the model.

### A1 — I3 now bounds wind work rather than forbidding it

**Original wording:** *"coasting/ballistic bodies monotonically lose energy to drag."*

**As ratified:** an unpowered body's mechanical energy may only fall, **except by at most the
work the wind could have done on it**, bounded by `(|F_drag| + |F_lift|) · |v_wind| · dt / m`.
With RB7 wind disabled the allowance is exactly zero and the strict form applies unchanged —
which is the case every golden trajectory runs under.

*Why.* The original wording was simply wrong when wind is on. Drag opposes the **airspeed**, not
the ground velocity (that is I5's own requirement), so a body being blown along genuinely gains
mechanical energy in the world frame. The wind does work on it, exactly as it does on a leaf.
Forbidding that would have forced the model to be wrong in order to satisfy the invariant.

*Found by:* the I8 fuzz harness, 317 breaches across 6,000 randomised flights.

### A2 — I3 is enforced by rotating lift, not by tolerating its error

**As ratified:** lift is applied as a **rotation of the velocity vector** through
`atan2(a⊥·dt, |v|)`, never as an addition. The parallel component, drag and gravity remain
additive.

*Why.* Adding a perpendicular acceleration in any Euler scheme manufactures energy
quadratically: `|v + a⊥dt|² = |v|² + |a⊥dt|²`. Usually that is lost in the noise. It is not lost
in the noise when `a·dt` is comparable to `|v|` — a 90 g airframe at 3 m/s was gaining **40 J/kg
per substep**. Lift does no work, so the honest integration is the one that says so exactly, at
any step size.

The substep planner gained a second bound at the same time: acceleration × substep may not
exceed 20% of the body's own speed. The displacement bound alone cannot catch this, because a
slow body under enormous acceleration covers almost no distance while its velocity vector swings
through most of a right angle.

*Found by:* the same fuzz sweep, after A1 removed the wind cases.

### A3 — TERMINAL is a powered phase, and `BOOST → TERMINAL` is legal

**As ratified:** `FlightPhase.isPowered()` is true for **BOOST and TERMINAL**, and BOOST may
transition directly to TERMINAL.

*Why.* The proximity fuse arms in TERMINAL. A short-burn interceptor — 1.25 s of motor, 1.3 s to
intercept — was still in BOOST at closest approach, so it flew through its target at **1.08 m**
with the fuse safed and was scored a 23 m miss. Real interceptors routinely arrive before
burnout; making the endgame wait for the motor to quit is the bug, not the symptom.

Two further corrections went in alongside it, both real and both found by golden scenarios:

- The **seeker was not updated during boresight alignment**, so PN started cold on a target it
  was already crossing. A head pointed straight at the target during an alignment turn is in the
  best position it will ever be in; skipping the look cost a 113 m miss.
- The alignment law was **pure pursuit**, which adds velocity toward the target but does nothing
  about the velocity the missile already has going elsewhere. It is now velocity-to-be-gained,
  solved against the missile's *expected* speed rather than its current closing velocity — the
  latter is near zero at launch for an off-boresight shot and returns an aim point in the wrong
  hemisphere. With this, a 120° off-boresight launch over 800 m hits at **0.4 mm**.

---

## Amendments ratified at the cosmos Phase B gate (v0.1.3)

### A4 — `LANDING` is a powered phase, and `DESCENT → LANDING` is legal

Landing on an airless world is not falling. With no atmosphere there is nothing to give energy
to, so a vehicle arrives at whatever speed gravity gave it unless an engine cancels it — and that
makes arriving intact a rocketry problem with a fuel bill, not a parachute problem.

`LANDING` therefore joins `BOOST` and `TERMINAL` as a phase in which thrust exists. It is entered
from `DESCENT` when the closed-form suicide-burn height is reached, and it exits to `LANDED`, to
`TERMINATED`, or back to `DESCENT` when the tanks run dry mid-burn — which is the honest outcome
of an under-fuelled lander and is exactly what the battery checks.

**`LANDING` is declared last in the enum, out of the flight's natural order.** Trajectory states
are hashed with the phase's ordinal in them, so inserting a phase mid-enum renumbers every phase
after it and invalidates every committed golden hash — not because a trajectory changed, but
because the numbering did. That happened once during this work and every golden failed at once.
Appending keeps the goldens meaningful: they fail if and only if the physics moved.

### A5 — a profile must have mass after its last stage is shed

`payloadDryMass` must be strictly positive, enforced at construction.

Kinetics deliberately sheds the final stage's structure at burnout, so a profile with no payload
mass ends its flight at **zero kilograms**, and the next force divided by that mass is `NaN`. I1
caught it — loudly, correctly, and about ninety seconds of simulated flight after the profile that
caused it went in. Rejecting it at construction turns a mid-flight P0 into an unmistakable message
naming the profile. There is no such thing as a massless vehicle.

## Amendment ratified at the cosmos economy gate (empire law)

### A6 — unattended simulation runs on the server tick, never on an entity tick

**Ratified as empire law, and it applies to every mod, not only to physics.**

Anything that must keep progressing while nobody is nearby must be driven by the server tick or
recomputed from an epoch. It may never hang off `Entity.tick()` or `BlockEntity.serverTick()`,
because those do not run for unloaded chunks — and a simulation attached to them does not fail
loudly, it silently does nothing while everything downstream reports success.

Kinetics already obeys this and should be read as the reference implementation:

- `KineticsService.tick` integrates every body from the **server** tick. A rocket flies whether or
  not anyone is watching it, and `RocketEntity` is explicitly a view — if it never ticks, the only
  thing lost is the plume.
- `OrbitalRegistry` goes further and **never accumulates**: satellite state is propagated from an
  epoch, so an orbit is correct whether it was ticked once, a thousand times, or not at all. State
  that cannot drift cannot drift while you are not looking (RE1).

This was learned three times in one campaign by consumers of this library — launch insertion,
capsule recovery, and the lunar ISRU roster — each time as a different-looking bug with the same
root. It is written into `mod-installer/CLAUDE.md` as rule 7 so there is no fourth.

## Open work (not v0.1.0 blockers)

- **GC tuning before a real combat load.** The 20-minute soak holds 0.74 ms mean and 1.39 ms p99
  against a 2 ms budget, but a single tick hit **34 ms** — a collection pause, not sustained
  cost. Under a real warfront engagement the allocation rate will be higher than the soak's.
  Investigate allocation in the hot path (`Vec3` is a record and every operation allocates) and
  whether the server's collector needs tuning, before the library carries live combat. Tracked
  as issue #1.

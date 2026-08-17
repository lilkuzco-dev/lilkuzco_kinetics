# lilkuzco_kinetics

The empire's motion authority. A server-authoritative physics library for Fabric / Minecraft 26.2
that owns how things fly — and nothing else.

Warfront's missiles, naval's shells, aircraft, and cosmos' rockets and satellites all consume it.
That is why it is a separate repository: one gravity, one atmosphere, one set of invariants, and
no mod quietly disagreeing with another about how fast something falls.

**Kinetics contains no player content.** No blocks, no items, no entities, no recipes, no worldgen.
It is a library packaged as a mod because that is how a server loads shared code.

---

## Build and test

Needs nothing but a JDK 25. No network, no Gradle, no JUnit:

```sh
tools/build.sh test          # the whole battery
tools/build.sh test golden   # one suite
tools/build.sh audit         # regenerate SCALE-AUDIT.md
tools/build.sh golden-record # re-record golden hashes (deliberately)
```

Or through Gradle, which also builds the mod jar:

```sh
./gradlew :core:battery      # the same battery
./gradlew :fabric:build      # -> fabric/build/libs/lilkuzco-kinetics-0.1.0.jar
```

The battery is a plain `main` class rather than JUnit, on purpose: `kinetics-core` has zero
dependencies so that nothing outside the module can perturb a trajectory, and a test runner that
dragged in a dependency tree would undercut the very claim it exists to verify.

---

## What is in here

| | |
|---|---|
| **[API.md](API.md)** | the surface consumers use |
| **[SCALE-AUDIT.md](SCALE-AUDIT.md)** | every constant that departs from reality, with the factor and the reason (generated) |
| `core/src/main/resources/physics-constants.json` | every physical constant, with SI units and provenance |
| `core/src/main/resources/profiles/` | eleven reference profiles, doubling as worked examples |
| `core/src/main/resources/golden-trajectories.txt` | committed trajectory hashes (generated) |

---

## The three ideas worth knowing

**One block is one metre and gravity is Earth's.** Almost nothing is scaled. Air is 1.225 kg/m³ at
sea level, drag coefficients are their published values, kerolox has 311 s of vacuum Isp. Exactly
three constants depart from reality, each buying something specific, and all three are in the
scale audit with their reasoning.

**The compressed atmosphere is the one thing that changes everything.** Real air has 8,500 m of
scale height; Minecraft has 257 m of buildable altitude. Folding one into the other makes altitude
a real tactical variable instead of decoration — and it has consequences the library states rather
than hides. The atmosphere here carries 1/155th of Earth's column mass, so a reentry vehicle needs
a genuinely large drag area for its mass. A capsule with a real-world ballistic coefficient arrives
at the ground still supersonic. The shield has to be big, and the physics says how big.

**Physics is enforced, not intended.** Twelve invariants gate CI. Several cannot be violated by any
code path — mass is derived rather than stored, drag is built as a negative multiple of the airspeed
unit vector, the event hierarchy is sealed and carries no damage field. The rest are checked every
substep, and a breach throws with the offending profile and full state attached, because "NaN
somewhere in flight" is not a diagnosable report.

---

## What the tests actually check

168 checks, ~25 seconds, no network.

**Closed-form cross-checks** compare what the simulation produced against the published equation,
computed independently — terminal velocity, drag-aware ballistic range, Tsiolkovsky delta-v,
altitude-varying Isp, orbital period and escape velocity, the rotating-frame ground-track shift, the
fourth-root radar law, the stall curve, the induced-drag polar, the transonic rise, and the
blunt-body reentry result.

**Fourteen golden trajectories** assert behaviour *and* hash every bit of every state. A physics
change that improves accuracy will break the hashes, and that is correct: they are re-recorded
deliberately, never adjusted to make a failing test pass.

**10,000 fuzz cases per run.** 4,000 deliberately malformed profiles, every one rejected at load
with an explanation; 6,000 valid-but-extreme flights — gram-scale bodies, hundred-tonne bodies,
vacuum and 1 g, wind on and off, guidance demanding hundreds of g — with zero invariant breaches
across 978,020 integrated body-ticks.

The fuzz harness earns its keep. It found two genuine physics bugs during v0.1: with wind enabled
drag really does work on a body and the strict energy invariant was wrong to forbid it, and a
perpendicular acceleration added by Euler manufactures energy quadratically, which lift must
therefore apply as a rotation of the velocity rather than an addition. Both are fixed in the model.

**Performance.** 500 guided bodies and 50 satellites, 24,000 ticks — a full twenty minutes of
simulated time — at 0.69 ms mean MSPT and 0.98 ms at the 99th percentile, against a 2 ms budget.

---

## Known work

**GC tuning before a real combat load** (issue #1, not a v0.1.0 blocker). The soak holds 0.74 ms
mean and 1.39 ms p99 against a 2 ms budget, but one tick in 24,000 hit 34 ms — a collection pause,
not sustained cost. `Vec3` is a record and every vector operation allocates, so the hot path's
allocation rate is high. Worth investigating before warfront puts a live engagement through it.

See `PHYSICS-CONSTITUTION.md` for the twelve invariants as ratified, including the three
amendments made at the v0.1.0 gate.

## Licence

MIT. All code original; no third-party code or assets. See `../ASSETS-ORIGIN.md`.

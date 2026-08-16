# API.md — lilkuzco_kinetics v0.1.0

The surface consumers use. Warfront, naval, aircraft and cosmos all talk to this and nothing
below it.

---

## The shape of the thing

Two modules, and the boundary between them is the point.

| Module | Depends on | Contains |
|---|---|---|
| **`kinetics-core`** | *nothing* — pure Java 25 on the standard library | All physics, all invariants, all profiles, the whole test battery |
| **`kinetics-fabric`** | Fabric Loader, Fabric API, `core` | One `WorldProbe` implementation, one service, one tick hook |

`core` has no Minecraft dependency and no third-party dependency of any kind. That buys three
things: the entire test battery runs headless in ~24 seconds with no server and no network;
consumers on different Minecraft versions can share the same physics; and determinism (I7) cannot
be perturbed by anything outside the module.

The bridge is a **single interface with one method of substance**:

```java
public interface WorldProbe {
    boolean isSolid(int blockX, int blockY, int blockZ);
    // + default lineOfSight(), groundHeight()
}
```

That is everything kinetics is permitted to know about the world. There is no handle here through
which damage could be applied even by accident, which is what makes **I10** enforceable by
inspection rather than by vigilance.

---

## Getting a body flying

```java
KineticsService kinetics = KineticsMod.service();

Profile profile = new ProfileLoader(Constants.get()).load(json);

KineticsService.Handle missile = kinetics.spawn(
        "warfront:sam_battery_3_shot_17",
        profile,
        level.dimension(),
        new Vec3(x, y, z),           // position, world coordinates
        new Vec3(vx, vy, vz),        // initial velocity, m/s
        FlightDirector.Mission.GUIDED);
```

`Mission` selects the guidance chain, never the physics:

| Mission | Chain |
|---|---|
| `BALLISTIC` | fired and forgotten; drag and gravity do the rest |
| `GUIDED` | boost (with off-boresight alignment) → midcourse PN → terminal + fuse |
| `LAUNCH` | gravity turn → staging → orbital insertion, or an honest ballistic failure |

The service ticks every live body on `END_SERVER_TICK`. Supply targets and countermeasures per
tick through the two functions on `tick(...)`; pass `null` for either if you have none.

---

## Events — the single damage door

Kinetics tells you what happened. It never decides what that means.

```java
kinetics.addListener(event -> {
    if (event instanceof KineticEvent.Impact impact) {
        AreaStrike.resolve(level, impact.position(), impact.kineticEnergy());
    }
});
```

`KineticEvent` is a **sealed** interface with 18 permitted types. A consumer cannot invent one,
and adding one is a visible API change. No event carries a damage, health or destruction field —
`InvariantTests` reflects over all 86 fields of all 18 types and asserts it.

| Group | Events |
|---|---|
| Flight | `Impact` `Proximity` `StructuralLimit` `ChuteShred` `ChuteDeployed` `ReentryOverheat` `Staging` `PhaseChange` `LiftoffFailure` |
| Seeker | `LockAcquired` `LockLost` `LockExpired` `DecoySeduced` |
| Orbital | `OrbitInsertion` `InsertionFailed` `OrbitDecaying` `Deorbit` |
| Failure | `InvariantBreach` |

`Impact` and `Proximity` expose `kineticEnergy()`, which is what a consumer scales its own effect
from. That is the correct shape for the single door: kinetics supplies the physics, the consumer
supplies the consequence.

---

## Profiles

A profile is what a consumer writes to describe something it wants to fly. JSON, schema-validated
at load, hot-reloadable, and the errors are addressed to whoever is editing the file:

```
profile 'cosmos:rocket_tier1': stages[0] declares vacuum Isp (280.0 s) below sea-level Isp
(320.0 s), which is thermodynamically backwards - a nozzle performs better in vacuum than
against ambient pressure, never worse. Check the two values are not swapped (RD2b).
```

```json
{
  "id": "cosmos:rocket_tier1",
  "payload_dry_mass": 100.0,
  "stages": [{
    "engine": "kerolox_first_stage",
    "fuel_mass": 16691.0, "stage_dry_mass": 2276.0,
    "thrust_vacuum": 380752.0,
    "isp_sea_level": 283.0, "isp_vacuum": 311.0
  }],
  "airframe": {
    "reference_area": 3.5, "cd0": 0.35,
    "wing_area": 0.0, "aspect_ratio": 2.0,
    "g_limit": 12.0, "q_max": 80000.0,
    "nose_radius": 0.9, "rcs": 12.0
  },
  "recovery": [
    {"name": "drogue", "cd": 1.40, "area": 10.0, "q_deploy_max": 18000.0, "deploy_altitude": 200.0},
    {"name": "main",   "cd": 1.65, "area": 45.0, "q_deploy_max":  5000.0, "deploy_altitude":  90.0}
  ],
  "seeker": {
    "quality": "ADVANCED", "field_of_view_deg": 45.0, "pn_gain": 4.0,
    "min_range": 15.0, "max_range": 400.0, "max_crossing_rate_deg": 120.0,
    "flare_resistance": 0.8, "chaff_resistance": 0.75
  }
}
```

**Specific impulse is declared as its real-world figure** — 311 s for kerolox — so engines stay
recognisable. The sim divides the resulting exhaust velocity by a documented scale; see
`SCALE-AUDIT.md`.

Eleven reference profiles ship in `profiles/kinetics-default.json` and double as worked examples.

---

## Reading the physics

Everything below is closed-form and free of side effects. Consult it, do not reimplement it.

| Question | Call |
|---|---|
| Will this rocket reach orbit, and why not? | `new Propulsion(k).assess(profile, gravity)` → a `LaunchAssessment` with a plain-English verdict |
| What elevation hits that point? | `new BallisticSolver(k).fireAt(profile, env, origin, target, muzzleSpeed, wind, highArc)` |
| How far can this radar see that? | `new Radar(k).detectionRange(rcs)` |
| Is this shot worth taking? | `new EngagementEnvelope(seeker).evaluate(pos, vel, speed, target)` |
| Where is that satellite, and when does it pass overhead? | `registry.stateAt(id, time)`, `registry.predictPasses(...)` |
| What is terminal velocity here? | `Aerodynamics.terminalVelocity(mass, g, rho, cd, area)` |

Example — the launch verdict, which is what a launch pad should show a player before ignition:

```java
var verdict = new Propulsion(k).assess(profile, env.gravity());
// "lifts off but falls short: 1360.6 m/s of ideal delta-v against a 2230.0 m/s budget,
//  869.4 m/s short. Add propellant, stage, or raise Isp."
```

---

## The orbital registry

Satellites are not entities. They are elements referred to an epoch, and every query recomputes
from that epoch — so an orbit advances at the same rate whether its chunk is loaded, whether the
server is lagging, and whether anyone has looked at it in a week.

```java
OrbitalRegistry registry = kinetics.orbits();

var result = registry.attemptInsertion(id, achievedDeltaV, worldTime,
                                       inclinationDeg, raanDeg, argLatDeg, events);
// short of the budget -> refused, no partial credit, the vehicle falls back

var state  = registry.stateAt(id, worldTime);        // position, ground track, decay status
var passes = registry.predictPasses(id, from, x, z, halfAngleDeg, 3, horizon);
var entry  = registry.deorbit(id, worldTime, true, events);  // -> in-world DESCENT body
```

Ground tracks are computed in the **rotating** frame. Predicting a second pass without that
correction is not slightly wrong, it is wrong by the whole angle the planet turned through, and
the error compounds every orbit.

---

## What kinetics will not do

Fenced permanently, and the fences are load-bearing rather than decorative:

- **No damage.** Events only. Consumers call `AreaStrike.resolve()`.
- **No rigid-body block assemblies.** The Valkyrien lane is closed.
- **No physics outside this library.** A consumer needing motion kinetics does not cover should
  propose an addition here, never write local math. Local math is how two mods end up with two
  different gravities.
- **No second energy system.** That is crude_empire's lane.
- **No worldgen.** Kinetics touches no terrain.

---

## Version and stability

`0.1.0`. The following are settled and safe to build against: `WorldProbe`, `KineticEvent`,
`Profile` and its JSON schema, `FlightPhase` and its legal transitions, `KineticsService`.

Known to change in `0.2`:

- **Elliptical orbits.** `OrbitalMechanics.visViva` is implemented and correct, but v0.1 only ever
  calls it with `r == a`. Hohmann transfers and eccentric orbits are fenced; the API is scaffolded
  so adding them is not a signature change.
- **Gravity falling with altitude.** In-world flight uses constant `g`, which is right to within
  0.0002% across the whole 257 m build range where atmospheric flight happens. It is *not* right
  for a launch vehicle still boosting at 80 km, where true gravity is about 35% lower — so v0.1
  slightly overestimates gravity losses on a long ascent. Stated rather than hidden.
- **Hyperbolic trajectories.** A launch with escape-velocity surplus is capped at a very high
  bound orbit and says so in the insertion detail.

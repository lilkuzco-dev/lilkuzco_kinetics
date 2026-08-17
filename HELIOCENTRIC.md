# HELIOCENTRIC.md — scoping the patched-conic frame

**Status: SCOPING DOC. Approved for scoping at the cosmos v0.1.0-D gate. Nothing implemented.**

This is the epic that would let the empire have planets rather than moons. It is scoped here on its
own before any of it is written, because it is the largest single change kinetics has contemplated
and because half of its value turned out to be somewhere other than where I expected.

## What the current frame is

One gravitating body. `OrbitalMechanics` holds a single `mu = g0 * R^2`, every orbit is around the
primary, and every transfer is a two-body Hohmann. That is exactly right for the Moon, which
genuinely is a satellite of the primary — and it is why the Moon needed no special cases anywhere.

## The measurement that motivates the change

| Destination | Distance | Injection Δv | Coast |
|---|---|---|---|
| Moon | 60.3 R | 732 m/s | 27.7 h |
| 314 R | 314 R | 750 m/s | 322 h |
| Mars analogue | 12,296 R | 754 m/s | 78,630 h |

**Every destination costs the same.** Correct for one body — past escape velocity the marginal cost
of going further collapses — and fatal for a rocket ladder.

## The finding I did not expect

I assumed the heliocentric frame would restore the Δv ladder. **It mostly does not.**

Real Earth→Mars injection from LEO, escape included, is about 3,600 m/s. At this world's velocity
scale of 4.2154 that is **854 m/s**, against the lunar TLI's **740 m/s**. Fifteen percent.

So a heliocentric frame does not make Mars dramatically dearer than the Moon in propellant. What it
makes Mars is:

- **Far in time.** 259 days of transfer against the Moon's 27.7 hours — 224× longer.
- **Available sometimes.** The synodic period is 780 days. You go when the window opens, or you do
  not go.
- **Hard to arrive at.** Capture and entry, not just a burn.

**That is the real prize, and it should be stated plainly rather than oversold.** If the goal is
"Mars costs a much bigger rocket", this epic does not deliver it and nothing honest will. If the
goal is "space has geometry, distance is time, and you plan around windows you do not control",
this delivers exactly that. I think the second is the better game and the first is a
misunderstanding of orbital mechanics that the current frame happens to share.

## The constraint this already imposes on today's work

Sphere of influence, `r = a·(m/M)^(2/5)`, for an Earth-analogue at 1 AU:

**924,646 km real = 145.1 planetary radii scaled.**

- The Moon at 60.3 R is **inside** it. Good — it is a real satellite and stays one.
- Anything beyond **145 R is not bound to the primary at all** and would have to be re-homed as a
  heliocentric object the moment this frame ships.

**So every in-system body built before this epic must sit inside 145 R.** The "near asteroid at
314 R" sketched in `cosmos/PLANETS.md` would have been wrong. That constraint is live now, and it
is the most useful thing this document has produced before writing a line of code.

## What would change in kinetics

### New

- **`GravitySystem`** — a tree of bodies: primary, satellites, and a central star. Each carries
  `mu`, orbital elements about its parent, and a derived sphere-of-influence radius.
- **`PatchedConic`** — transfer planning across SOI boundaries: escape the departure body's SOI on
  a hyperbola, coast a heliocentric ellipse, arrive on a hyperbola at the destination.
- **`TransferWindow`** — synodic periods and phase angles. Answers "when is the next window, and
  what does it cost". This is the feature, more than the transfer itself.
- **`Capture`** — arrival: a burn, or aerocapture where there is atmosphere.

### Unchanged, and this is the point

`Integrator`, `PoweredDescent`, `FlightDirector`, the phase machine, `Atmosphere`, `WindField`,
`Seeker`, `ProportionalNavigation` and every invariant I1–I12 are **two-body or local**. They
survive untouched and get applied per patch. The epic adds a layer above them; it does not
reach into them.

### Constants that finally become derived

`orbit.lunar_orbit_insertion_delta_v` and `orbit.lunar_descent_delta_v` are today **scaled from
reality, not derived**, because deriving them needs a lunar μ and radius — a second gravitating
body. Their source notes say so explicitly. **This epic is what lets them become derived, and those
two notes are the acceptance checklist.**

## Invariants this must add

Written in the style of I1–I12 so they can be enforced as build-failing gates:

- **I13 — patch continuity.** Position and velocity are continuous across an SOI boundary. A
  transfer that teleports by a metre at the patch is a bug, not a rounding.
- **I14 — energy across a patch.** Specific orbital energy relative to the *new* primary must equal
  what the hyperbolic excess implies. No energy may be gained by changing which body you are
  measured against.
- **I15 — window determinism.** The same epoch and the same system produce the same window, always.
  A launch window that moves is not a window.
- **I16 — no unbound satellites.** A body registered as orbiting a parent must lie inside that
  parent's SOI. This one is checkable *today* and would have caught the 314 R mistake.

## What must not break

**Every golden trajectory must still hash identically.** All of them are two-body flights within
one SOI, so they must be untouched by definition — and if any hash moves, the layering is wrong and
the change has reached somewhere it should not have.

That is the strongest available acceptance test, and it is free.

**Watch the enum ordinals.** Adding a `FlightPhase` mid-enum renumbered every phase after it and
invalidated every golden hash — not because a trajectory changed, but because the numbering did.
`LANDING` is declared last for that reason. Any new phase this epic needs is appended, never
inserted.

## Phasing

1. **`GravitySystem` and SOI, read-only.** No transfers. Register the existing primary and Moon,
   assert I16 against them. Zero behaviour change; the goldens prove it.
2. **`PatchedConic` closed-form.** Transfer planning and Δv only, cross-checked against published
   Earth→Mars figures. Still no flight.
3. **`TransferWindow`.** Synodic periods and phase angles, with I15.
4. **Flight across a patch**, with I13 and I14.
5. **Capture and aerocapture.**

Phases 1–3 are pure additions with no way to break an existing flight. Phase 4 is where the risk
is, and it is the first phase where a golden could move.

## Open question for the ruling that follows

`cosmos/PLANETS.md` still carries it: **hard launch windows or soft?** Hard means you wait, which is
what makes a window a window. Soft means you can always go, badly. I lean hard, because a window
you can ignore is a number on a screen — but it is a server-social decision, not a physics one.

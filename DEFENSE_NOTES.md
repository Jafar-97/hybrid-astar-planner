# Defense notes — how to talk about this project

This is the "walk me through it" cheat sheet. Read it until you can explain each
answer in your own words. If you can defend these, you can hold a real
conversation about the code with an autonomy engineer.

## The 30-second pitch

> "I built a Hybrid A\* motion planner in C++ that produces kinematically
> feasible paths for a car-like vehicle, plus a pure-pursuit controller that
> tracks them in a closed-loop bicycle-model simulation. It plans through a
> costmap in ~150 ms and tracks the path to under 10 cm cross-track RMS. It's
> fully reproducible — two benchmark scenarios, unit tests, and a visualizer,
> no ROS or external datasets."

## Core concepts — be ready for these

**Q: Why Hybrid A\* instead of plain A\* or RRT?**
Grid A\* plans over cell centres, so its paths have 90°/45° corners a car can't
drive — you'd need a separate smoothing/feasibility stage. Hybrid A\* associates
a continuous vehicle state (x, y, heading) with each grid cell and expands nodes
by *integrating the motion model*, so every edge is already drivable. Versus
RRT/RRT\*: sampling planners are great in high-dimensional spaces but give
non-deterministic, jagged paths that need heavy smoothing; Hybrid A\* is
deterministic and resolution-complete, which is what you want for structured
driving. This is the planner from the DARPA Urban Challenge (Dolgov, Thrun et al.).

**Q: What makes the heuristic admissible, and why two of them?**
`h = max(h_nonholo, h_holo)`. `h_nonholo` is the Dubins length — the true
shortest path for a bounded-curvature car in free space, so it never
overestimates. `h_holo` is a Dijkstra field over the actual costmap ignoring
heading — also a lower bound on remaining cost. The max of two admissible
heuristics is still admissible, and each covers the other's weakness: Dubins
knows nothing about walls; Dijkstra knows nothing about the turning radius.
Combining them is what keeps node expansions in the tens of thousands instead of
millions.

**Q: What is the analytic expansion and why does it matter?**
Discrete steering primitives rarely land *exactly* on the goal pose. Every few
expansions I try a single Dubins curve from the current node straight to the
goal; if it's collision-free, I splice it on and finish on the exact goal
heading. It also short-circuits the search dramatically once you're in the goal's
neighborhood. Standard Hybrid A\* practice.

**Q: What's the bicycle model?**
A car simplified to two wheels on an axle of length L. State (x, y, θ);
`θ̇ = v·tan(δ)/L` where δ is the steering angle. It captures the key constraint —
bounded curvature (minimum turning radius R = L/tan(δ_max)) — without full tire
dynamics. The planner and the simulator use the identical model, which is *why*
the plan is trackable.

**Q: How does collision checking work?**
The costmap is inflated by half the vehicle width plus a margin, so I can model
the footprint as a chain of discs along the centreline and reduce each to a
point-in-cell test. It's conservative (never reports a collision-free pose that
isn't) and fast. The tighter alternative is a swept-polygon check — a known
next step.

**Q: Why pure pursuit for control? What are its limits?**
Pure pursuit picks a look-ahead point on the path and geometrically solves for
the steering that arcs the vehicle onto it: `δ = atan2(2·L·sin(α), L_d)`. It's
simple, robust, and industry-standard for lower speeds. The catch is the
look-ahead distance `L_d`: too long and it cuts corners on tight turns (I hit
exactly this — cross-track blew up to 6 m until I shortened `L_d` below the
minimum turning radius and slowed the car in curves); too short and it
oscillates. For high speed / aggressive maneuvers you'd move to an optimization
controller — **MPC** — that respects dynamics and actuator limits over a horizon.

**Q: What do your metrics mean?**
*Plan time* — wall-clock for the search. *Nodes expanded* — search effort, the
thing the heuristic minimizes. *Path length* — solution quality. *Cross-track
RMS/max* — how tightly the controller follows the plan; sub-10 cm RMS means the
plan is genuinely feasible and the controller is well-tuned.

## Trade-offs I made deliberately (and would call out)

- **Forward-only (Dubins), not Reeds–Shepp.** Reverse motion needs Reeds–Shepp
  curves and a direction-change penalty. I scoped it out to keep the core clean;
  it's the #1 extension and I know exactly where it plugs in (the analytic
  expansion + primitive set).
- **Self-contained sim instead of CARLA.** A CARLA integration looks flashier
  but buries the algorithm in glue code. A transparent bicycle-model sim makes
  the planner/controller behavior legible and the whole thing reproducible in
  one build. I'd frame CARLA as an integration step, not a core one.
- **Disc-footprint collision** over swept-polygon — speed and simplicity now,
  precision later.
- **Dependency-free test harness** instead of pulling gtest — keeps the build
  trivial to reproduce.

## If they push: what would you do next?
Reeds–Shepp for reverse → swept-volume collision → path smoothing (CHOMP-style)
→ replace pure pursuit with a lateral MPC → a time-indexed costmap for dynamic
obstacles. In that order, because each unblocks a class of scenarios the last
one couldn't handle.

## Honesty guardrail
Don't claim this is production AV code or that you invented Hybrid A\*. It's a
faithful, well-engineered implementation of a known algorithm with measured
results — that's exactly what's impressive for an early-career candidate. Say
that plainly if asked. Overclaiming is the one thing that will sink you.

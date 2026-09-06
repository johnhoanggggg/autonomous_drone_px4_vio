# Corner weave: diagnosis and the join lateral cap — 2026-09-06

Flights `offboard_global_20260906T120129Z` (goal reached, 38.4 s of route) and
`offboard_global_20260906T120238Z` (10.9 s of route; the bag's last record is
truncated). Both ran the C++ adapter with the fixed path-tangent heading source.

## Reported symptom

Goes too far left entering a right-hand turn, then overcorrects right. Wants to
fly down the centre of the tunnel.

## What the logs show

It is not tracking error and not the nose. The vehicle follows its setpoint to
within about 1.5 degrees, and there is no body-fixed drift bias (velocity
residual mean 0.02 m/s, no consistent sign by heading octant). The commanded
velocity itself points sideways.

Angle between the commanded velocity and the route tangent at the command point:

| | on the accepted path | while rejoining |
|---|---|---|
| 12:01:29Z | median 0.0°, p90 0.0° | median **+18°**, p90 54°, worst 85°, 57% of route |
| 12:02:38Z | median 0.0°, p90 0.0° | median **+27°**, p90 68°, worst 83°, 36% of route |

Positive is left of the route — the outside of these right-hand bends.

The loop that sustains it:

1. The planner republishes about twice a second, each path anchored at the
   vehicle's current position.
2. The command point is not at the vehicle, so the fresh path lands a few
   centimetres to one side; anything past `join_tolerance_` (3 mm) opens a join.
3. `update_join()` closed that offset at
   `min(max_speed, offset/dt, sqrt(2*a*offset))`. At 0.30 m/s² anything past
   ~7 cm saturates at the full 0.20 m/s cruise, and an equal along-path term
   puts the commanded velocity 45° off the route at full speed.
4. The vehicle follows it and leaves the line, outward on a bend.
5. The next path anchors on the drifted vehicle; back to 2, further out.
6. Cross-track passes the follower's `max_cross_track` 0.08 m, the follower goes
   invalid, and the adapter HOLDs the command with no feedforward — the snap
   back. 13 trips across the two routes.

Separately: with `junction_deviation` 0.05 m and 0.30 m/s², no corner under
about 90° slows the command at all, so the 20–45° bends on these routes are
taken at full cruise with a 5 cm deviation budget the vehicle's own lag spends.

## The fix (implemented)

`PathCommandLimiter::update_join()` now caps the lateral closing rate against
the along-path rate: `along_speed * join_lateral_ratio_`, with
`join_lateral_ratio_ = 0.2679` (tan 15°). The commanded velocity therefore stays
within 15° of the route while a replacement is merged onto. `join_lateral_floor_`
(0.02 m/s) applies only when the along-path rate is exactly zero — a rejoin
opened where the route has no travel left — because a floor applied whenever the
along-path rate is merely small dominates the direction coming out of a brake,
where one tick of acceleration is the whole along-path rate. (Measured: with the
floor unscoped, the replay reported 53.1° = atan(0.02/0.015) on every bag.)

C++ only. `planner_flight.py` keeps the older frozen-target join; Python is
legacy and is not being kept in step.

At the cap, a 5 cm offset merges over about 0.19 m of travel (0.05/tan15°)
instead of darting across in half a second.

### Geometry replay over five recorded flights

Worst commanded deviation from the route while rejoining, before → after:

| bag | before | after (max) | after (p90) | samples over 20° |
|---|---:|---:|---:|---:|
| 20260831T082507Z | 84.0° | 32.6° | 15.2° | 4 / 62 |
| 20260831T064236Z | 85.6° | 49.2° | 15.0° | 9 / 176 |
| 20260831T064828Z | 84.5° | 52.9° | 16.7° | 10 / 125 |
| 20260906T113122Z | 84.2° | 15.0° | 15.0° | 0 / 35 |
| 20260906T113239Z | 85.7° | 15.0° | 15.0° | 0 / 56 |

The p90 sits on the cap everywhere. The residual maxima on the Aug 31 bags are
acceleration-limited rotation transients — the command velocity taking a few
ticks to swing onto a new leg direction — not the lateral term; those bags
replanned hardest.

**One new rejection.** 20260831T064236Z now reports a single
`path rejoin would leave the projection tolerance band` (1 preview, 1 update, out
of 554 samples); it reported none before. A slower merge keeps the offset alive
longer, so it is more likely to still be there when the command point crosses a
vertex. The adapter already treats this as a deferral: it restores the snapshot
and holds that tick, and the planner republishes within 0.5 s. Judged preferable
to lifting the cap for the largest offsets, which are exactly the ones that
caused the weave. Watch it in flight.

### Regressions added

- `JoinMergesInsideTheCappedAngleAndStillFinishes` — offsets 0.02/0.035/0.05,
  every commanded velocity within 16° of the route, join completes under 4 s.
- `JoinFinishesWithNoAlongPathMotionLeft` — the terminal case the floor exists
  for.

`colcon test`: 596 tests, 0 errors, 0 failures, 3 skips.

## Not done

- Path re-anchoring. A republished path differing only by the centimetres the
  vehicle has moved is still treated as a replacement to merge onto, so 36–57%
  of the route is still spent rejoining. Widening what counts as identical
  geometry, or carrying the command point's own offset onto the new path, would
  remove most of it. The cap makes each of those joins harmless; it does not
  reduce how many there are.
- `junction_deviation`. Untouched, so 20–45° bends still get no speed budget.
- How much bulge is left from plain tracking lag once the commanded direction is
  capped. That needs a flight, not another pass over these bags.

Nothing here is closed-loop. Props-off `auto_arm:=false` with `cpp_mode:=true`
first, then a low-speed flight with an accessible RC kill.

---

# Second iteration: the flat cap landed an aircraft — 2026-09-06

Flights `offboard_global_20260906T124409Z` and `T124502Z` flew the flat 15° cap.

## The cap worked

Commanded velocity against the route tangent, while a rejoin is open:

| flight | median | p90 | max | commands >20° off route |
|---|---:|---:|---:|---:|
| 120129Z (before) | 29.6° | 43.6° | 76.8° | 42% |
| 120238Z (before) | 31.8° | 55.2° | 72.3° | 33% |
| 124409Z (after) | 15.0° | 17.3° | 32.5° | 4.5% |
| 124502Z (after) | 15.0° | 15.0° | 28.8° | 1.9% |

## And it landed 124409Z

The cap bounds the merge *rate*, not the time it takes, so an offset now stands
for seconds instead of being snapped out. The command point's `path_offset` rose
from a 0.006 m median before the cap to 0.035 m after, with maxima of 0.178 m
and 0.194 m.

1. 19.26 s: the planner replanned hard (4 → 7 points, first vertex moved
   0.23 m), leaving the command 0.168 m off the new path.
2. 15° of a 0.20 m/s cruise closes 0.054 m/s, so that needed about three
   seconds.
3. 19.92 s: the vehicle followed the command out past the follower's
   `max_cross_track` 0.08 m and the follower went invalid.
4. A latched fault means `update_route_command()` is not called at all
   (`offboard_global_planner_node.cpp`), so `set_path` **and** the merge both
   stop. The command point kept its 0.168 m offset and the vehicle held.
5. Map corrections were swinging in the same window — x 0.025 → 0.201 → 0.120 →
   0.029 and yaw +0.2° → −3.0° → +0.6°, a 0.17 m swing, while the vehicle
   physically moved 0.086 m — repeatedly throwing cross-track back to
   0.13–0.21 m and denying the follower the 0.6 s of quiet it needs.
6. 25.97 s: `planner fault persisted: follower validity is false -> AUTO.LAND`.

`124502Z` hit the same thing three times and recovered each time on a replan; it
also reached 0.211 m of offset, past `max_connector_error` 0.200 m, where no
replacement can be joined at all. Its bag ends truncated mid-hold at 32.9 s.

## Fix: escalate the angle with the offset

`update_join()` now widens the merge angle in proportion to how far past
`max_projection_error_` the offset has got:

```
escalation   = max(1, offset / max_projection_error_)
lateral <= along_speed * join_lateral_ratio_ * escalation
```

Inside the projection tolerance — which covers the re-anchoring offsets the
bound exists for, median 0.022 m and 0.048 m on the two weave flights — nothing
changes. Beyond it the command is somewhere a replacement can no longer be
joined from, and clearing that is worth a wider angle.

**A merge-*time* bound was tried first and rejected on measurement.**
`lateral >= offset / 1 s` is an absolute rate, so wherever the along-path rate is
small it dominates the direction — the identical failure the bound exists to
prevent. The replayed 90th percentile went straight back to 22–75°. The
escalation stays proportional to the along-path rate for exactly that reason.

### Measured

At the flown configuration (0.20 m/s, 0.30 m/s², 0.05 m projection tolerance) a
0.170 m offset is back inside the follower's 0.08 m limit in **1.05 s**, against
about 3 s under the flat cap, and fully merged in under 3 s. It is slower than
the bound alone implies because the along-path and lateral terms share one
`max_speed_` budget and the velocity still has to rotate under the acceleration
limit.

Geometry replay, worst / p90 / share over 20° while rejoining:

| bag | flat cap | escalating |
|---|---|---|
| 20260831T082507Z | 32.6° / 15.2° / 4 of 62 | 32.6° / 16.1° / 4 of 62 |
| 20260831T064236Z | 49.2° / 15.0° / 9 of 176 | 49.6° / 15.0° / 7 of 174 |
| 20260831T064828Z | 52.9° / 16.7° / 10 of 125 | 52.9° / 16.7° / 10 of 125 |
| 20260906T113122Z | 15.0° / 15.0° / 0 of 35 | 15.0° / 15.0° / 0 of 35 |
| 20260906T113239Z | 15.0° / 15.0° / 0 of 56 | 32.6° / 28.5° / 12 of 56 |
| 20260906T124409Z | — | 20.4° / 17.6° / 1 of 94 |

124409Z is the meaningful row: it is the only bag flown *with* a cap, so its
offsets are the ones a capped flight actually produces, and the escalation
almost never fires on them. The bags flown uncapped carry the large offsets the
weave itself created, which is why the escalation fires there.

Rejections rose from 2 to 4 on 064236Z. 124409Z replays 7 `set_path` refusals at
0.20–0.27 m — the recorded flat-cap drift, which the escalation exists to stop
the command reaching.

### Regression added

`JoinEscalatesToCloseALargeOffsetInBoundedTime` — flown configuration, 0.170 m
offset installed through the connector branch: back inside 0.08 m within 1.2 s,
merged within 3 s, and the angle escalates past the flat bound without reaching
the uncapped 45°+. `colcon test`: 597 tests, 0 errors, 0 failures, 3 skips.

`scripts/replay_command_heading.py` no longer treats a headingless rejoin as an
error: a rejoin whose along-path rate is zero still moves on the lateral term
alone, and that case is deliberately headingless — no route travel to face, so
the adapter keeps the yaw it has.

## Still open, and this matters more than the tuning

- **A latched fault freezes the limiter.** `update_route_command()` is skipped
  entirely while a planner fault is latched, so neither a replacement nor the
  merge can run, and a standing offset cannot close. That is what turned a
  cross-track trip into a landing. Ticking the merge (lateral only, no
  along-path advance) during a fault hold would let it recover; that is a change
  to the node's fault branch, not the limiter.
- **Map correction jumps.** 0.17 m of map shift and 3° of yaw on a stationary
  vehicle will break any cross-track logic regardless of the limiter.
- Re-anchoring, and `junction_deviation`, as before.

---

# Third iteration: the escalation held, the fault hold did not — 2026-09-06

Flights `20260906T131828Z`, `T131911Z`, `T131950Z`, flying the offset-proportional
escalation.

## The escalation fixed the ratchet

Command point's offset from the published path:

| flight | median | p90 | max |
|---|---:|---:|---:|
| 124409Z (flat cap) | 0.035 | 0.077 | 0.178 |
| 131828Z | 0.000 | 0.008 | 0.036 |
| 131911Z | 0.000 | 0.017 | 0.030 |
| 131950Z | 0.004 | 0.083 | 0.167 |

Merge angle median 15.0° on all three; commands more than 20° off route 1.2%,
1.1% and 20.1%. Vehicle cross-track max 0.063, 0.074 and 0.189 m — the two short
flights never approached the follower's 0.08 m limit.

## All three still ended early

- `131828Z` — feature watchdog, `latest=125`, 5.5 s into the route.
- `131911Z` — feature watchdog, `latest=122`, 6.3 s in.
- `131950Z` — `planner fault persisted -> AUTO.LAND` at 47.3 s after 35.5 s of
  route, with 0.69 m left to the goal.

### min_vio_features 150 -> 75

Feature counts during the three routes: min 107 / 111 / 125, median 269–296.
Below 150 for 1–7% of samples, below 100 for 0%, below 75 for 0%. The watchdog
was landing healthy flights on brief dips. 75 keeps margin against a real
collapse — `131911Z`'s bag does contain a sample at 18, outside the route.

### The fault hold pinned the aircraft inside a clearance violation

`131950Z`'s final fault is `CLEARANCE_ESCAPING`, holding between 0.119 m and
0.241 m of an obstacle needing 0.250 m, for about 4.5 s. Map corrections were
well behaved (x −0.13..−0.02, yaw 1.2–4.9°, drifting, no jumps), so unlike
`124409Z` they are not implicated.

Reading `latch_fault_hold()` corrected the earlier account of the deadlock: it
does not merely freeze the route limiter, it calls `route_limiter_->clear()` and
resets the published-command limiter to the vehicle pose. Discarding the
unvalidated route state is deliberate and right — a recovery restarts from the
actual pose. So there was never a standing merge to finish. What is wrong is
narrower and worse: the hold pins the aircraft exactly where it stands, and when
what it is standing in is a clearance violation, that is the one place it must
not stay.

## Fix: let a hold walk out of a clearance violation

New `best_clearance_escape_step()` in `route_follower.cpp`: given a pose with no
route command to escape along, try 16 headings at `fault_escape_step` (0.05 m)
and return the admissible one gaining the most clearance. Admissibility is the
existing rule via `command_chord_admissible()` — no point of the chord closer to
an occupied cell than the pose already is, and a material gain at the endpoint —
so it can only ever propose a step away.

`escape_clearance_while_held()` in the adapter drives the published-command
limiter toward that step while a fault holds. Bounded on every side:

- only in `State::Route`, armed, with a fresh pose, correction and grid;
- only while pose clearance is below the follower's required clearance;
- travel capped at `fault_escape_limit` (0.50 m) from where the hold began;
- the step is still speed- and acceleration-limited by the command limiter, and
  clamped into the geofence disc;
- the route is not resumed, and **the land countdown keeps running** — this buys
  the escape a chance to work, it does not excuse the fault.

The hold status now records `clearance=<pose>/<required>m escaping` or
`no admissible escape`, so a bag shows which it was.

### Regressions added

`HeldPoseStepsAwayFromTheWall` (steps away, and walking it repeatedly recovers
to the required clearance rather than sliding along the contour),
`HeldPoseProposesNothingWhenAlreadyClear`, `HeldPoseRejectsBadInputs`.
`colcon test`: 600 tests, 0 errors, 0 failures, 3 skips.

## Still open

- Not flown. Props-off `auto_arm:=false` first: the escape only runs armed and
  in ROUTE, so a props-off dry run exercises the decision, not the motion.
- The clearance the escape is escaping is the map-starvation problem — 0.119 m
  against 0.250 m required in a corridor this tight is a mapping and route
  question, not a control one.
- Re-anchoring at ~2 Hz, and `junction_deviation`, as before.

---

# Fourth iteration: two flights reached the goal — 2026-09-06

`20260906T135902Z` and `20260906T140117Z` both flew the escalating join cap, the
75-feature watchdog and the escape-while-held, and **both reached the goal** —
the first completions since 120129Z. `20260906T140001Z` in between was killed at
4.2 s before ENGAGE and carries no route. Overview renders are written into each
bag folder as `overview.png`.

| flight | route | net VIO | joining median / p90 / max | >20° off route | cmd offset p90 | veh cross max | HOLD share | outcome |
|---|---:|---:|---|---:|---:|---:|---:|---|
| 120129Z (pre-cap) | 38.4 s | 2.60 m | 29.6° / 43.6° / 76.8° | 41.7% | 0.048 | 0.223 | 36% | goal |
| 131950Z (escalation) | 35.5 s | 2.06 m | 15.0° / 28.9° / 44.0° | 20.1% | 0.083 | 0.189 | 37% | fault land |
| 135902Z | 32.8 s | 2.68 m | 15.0° / 18.5° / 29.1° | **2.7%** | 0.059 | 0.118 | 16% | **goal** |
| 140117Z | 26.5 s | 2.51 m | 15.0° / 22.4° / 31.6° | **7.3%** | 0.051 | 0.166 | 28% | **goal** |

Vehicle cross-track maxima fell to 0.118 m and 0.166 m, and the share of the
route spent in a fault HOLD roughly halved against the pre-cap flight.

## The feature threshold is what let them finish

Both flights had continuous spells below the old 150 threshold that outlast the
0.25 s watchdog:

| flight | min | median | longest run below 150 | below 100 | below 75 |
|---|---:|---:|---:|---:|---:|
| 135902Z | 91 | 269 | **0.68 s** — would have landed | 0.13 s | 0.00 s |
| 140117Z | 118 | 268 | **0.42 s** — would have landed | 0.00 s | 0.00 s |

At 150 both of these would have auto-landed mid-route. At 75 neither comes near
it — nothing even spent 0.15 s below 100 — so the margin against a genuine
tracking collapse is intact.

## The clearance escape did not run

Neither flight produced a single `CLEARANCE_ESCAPING` or `CLEARANCE_BLOCKED`
sample, and no hold status carried a `clearance=` line. Pose clearance never fell
below the requirement, so `escape_clearance_while_held()` correctly did nothing.
**It remains unexercised in flight.** Its holds were cross-track and
correction-settling, which it deliberately does not act on.

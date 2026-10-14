# Hybrid DRL + Nav2 (MPPI) local controller, 2026-10-11

House (assistive_world, no pillar), baseline protocol: fresh start, run A = 1 pass of the 6 goals, back to (0,0), run B =
2 passes; success = SUCCEEDED and Gazebo ground-truth distance <= 0.25 m; timeout 150 s. Same Nav2 overrides for all rows
(cm_reverse_override.yaml). Wilson 95% intervals. Back-to-back trials, one run each.

## Result

| Controller | Successes | Mean time (successes) | DRL share of distance |
|---|---|---|---|
| Nav2 MPPI (`nav2_cmrev`) | 14/18 = 78% [55, 91] | 84 s | 0% |
| **Hybrid DRL + MPPI + escape (`hybrid_run3_esc`)** | **13/18 = 72% [49, 88]** | **73 s** | **25% (45 of 181 m)** |
| Pure DRL run 3 (`drl_run3_cmrev3`) | 4/18 = 22% [9, 45] | 74 s | 100% |

Per goal (hybrid / Nav2): c1 3/3 / 3/3 (43 s vs 67 s), c2 3/3 / 2/3, c3 3/3 / 3/3, c4 3/3 / 3/3, c5 0/3 / 0/3, c6 1/3 / 3/3.
The hybrid is statistically level with Nav2 (intervals overlap almost completely) and faster on successful goals.
Its losses are c5 (goal 0.40 m from the wall; Nav2 also 0/3) and c6 twice, right after c5.

## How it works (training/deploy/hybrid_controller.py; pure DRL node drl_controller.py unchanged)
- Serves Nav2's FollowPath action like the DRL node; planner, BT, recoveries, smoother, Collision Monitor unchanged.
- DRL (run 3 actor) drives by default. Nav2's MPPI (controller_server, action follow_path_mppi_unused) takes over when the
  next 2 m of the planned path pass a narrow region (static-map clearance < 0.9 m: doors, gaps, near-wall goals) or a scan
  point comes within 0.25 m of the footprint. DRL takes back over when the path ahead is open again (>= 2 s in MPPI,
  scan clearance > 0.40 m, map clearance >= 1.0 m).
- Escape (at most 2 per navigation) before reporting "no progress" to the BT: reverse 0.30 m at 0.10 m/s if the scan
  shows > 0.35 m free behind, then turn toward the path while the turning circle is clear. 22 escapes in this run, most at
  the c5 corner (4.6-5.0, 2.3-2.6). Nav2's own BackUp was refused there by the costmap ("Collision Ahead").
- Run: DRL_NODE=training/deploy/hybrid_controller.py DRL_ARGS="--g-in 0" NAV2_OVERRIDES=cm_reverse_override.yaml MODE=drl
  training/deploy/compare_protocol.sh TAG

## Development steps (each measured on the same protocol)
1. Handover on scan clearance < 0.25 m only (smoke test): too late; DRL had already put the rover half into the gap, MPPI
   could not recover. Not run on the full protocol.
2. + map look-ahead, MPPI also for the last 1 m (`hybrid_run3_A_*_PARTIAL_gin1.csv`, run A only): 1/6; four goals ended
   0.10-0.31 m away because cold-started MPPI crept and aborted on the final approach. Stopped after run A.
3. Final approach back to DRL (`--g-in 0`, `hybrid_run3_gin0_*`): 5/18 = 28%; c1-c4 OK, then stuck after c5 (MPPI and BT
   recoveries made no progress facing the wall).
4. + scan-based escape (`hybrid_run3_esc_*`): 13/18 = 72%.

## Honest caveats
- DRL drives 25% of the distance (corridors and open room areas); MPPI drives doors, gaps and the near-wall goal. Per goal
  0-33% (more where the route is open).
- The escape is a scripted recovery, not learned. It is what turned 5/18 into 13/18.
- One run of 18 back-to-back trials. Not repeated, not tested on the generated worlds.
- Contact data not validated; do not claim collision-free.

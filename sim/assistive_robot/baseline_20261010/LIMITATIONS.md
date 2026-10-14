# Limitations of the 2026-10-10 Nav2 baseline

Read this before quoting the README numbers.

## Evaluation design
- Small sample: 6 goals, 3 passes, 18 attempts. Wilson 95% interval for 12/18 is 44-84%.
- Not independent trials: goals run back-to-back, each starting where the previous ended. A failure changes the next start pose (e.g. pass A candidate_6 started from where candidate_5 timed out and never reached Room 1).
- One world (the same one the goals were picked in). No held-out layouts.
- Collisions are not measured. /rover_contacts is unvalidated; "0 contact trials" is not a collision-free claim.
- Success needs both Nav2 SUCCEEDED and ground truth <= 0.25 m. Timeouts that ended within 0.25 m (candidate_1 at 0.15 m and 0.22 m) count as failures.

## Known failure modes
- candidate_5 (5, 2) failed in all 3 passes (and in the 23:06 pre-freeze run, by 1 cm). The goal is 0.40 m from the corridor_top_right wall face (y = 1.6). With the real 0.38 m half-width and goal yaw 0, the rover's side would be about 2 cm from the wall, i.e. inside the high-cost inflation band. It stops 0.23-0.34 m short after 13-14 recoveries. This looks like a goal that is too tight for the real footprint, not a planner failure. Not changed here, to keep the goal set fixed.
- Final-approach stalls: candidate_1 (and candidate_6 in earlier runs) stop 0.15-0.35 m short and loop "Failed to make progress" about every 10 s. That interval matches progress_checker (0.20 m in 10 s). Hypothesis only, not tested.
- Room 1 goals (candidate_1, candidate_6) are the least reliable apart from candidate_5.

## Localization
- AMCL at rest is about 0.08 m and about 4-6 deg off ground truth; while driving mean yaw error about 3-4 deg, peaks about 20-29 deg in fast turns (time-aligned probe).
- Odometry over-reports rotation by about 6-9% (wheel_separation tuned to 1.46; not tuned further).
- Nav2's goal checker (xy 0.15 m) uses the AMCL pose, so a goal can be "reached" while the real distance is 0.2-0.26 m.

## Costmap
- The global costmap obstacle layer marks scan hits with the AMCL pose; yaw error misplaces far hits. With obstacle_max_range 10 m one trip left 1209 phantom lethal cells on open floor; with 3.5 m, 10 (same probe, one run each). Reduced, not eliminated: AMCL error is unchanged.
- SmacPlanner2D logs "inflation is not set sufficiently for optimized non-circular collision checking" at startup, although inflation 0.65 m exceeds the footprint's circumscribed radius (0.59 m). Cause not investigated; it is a performance warning, planning worked.
- Collision Monitor stop polygon half-width 0.33 m is narrower than the rover (0.38 m). Not changed.

## Environment
- A Wi-Fi drop on 2026-10-10 (00:05-00:09) stalled ROS 2 DDS and Gazebo transport on the same machine; Nav2 shut itself down and the rover drove into bed_room1 with the safety chain down. That run is excluded (results/nav2_eval_v2_20261010_000519_INVALID_wifi_drop.csv). All baseline runs used localhost-only discovery (ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST, GZ_IP=127.0.0.1), which has not been tested against a real Wi-Fi drop.
- Gazebo's drive plugin probably keeps the last velocity command when comms stall (not verified).

## Pre-freeze history (not part of the baseline numbers)
- 16:15 run (old footprint 0.60 m, 0.9 m gap): 4/6; candidate_4/5 stuck at the gap.
- 23:06 run (gap 1.5 m, footprint 0.76 m, obstacle range 10 m): 4/6.
- Old V9/SAC numbers are not comparable (buggy reset) and are not quoted.

# Afternoon summary, 2026-10-11 (queue 16:27-17:35)

Same house protocol and Nav2 overrides as the morning runs. Wilson 95% intervals. Back-to-back trials.

| Run | Successes | DRL share of distance | Escapes |
|---|---|---|---|
| Nav2 MPPI, run 1 (morning, `nav2_cmrev`) | 14/18 = 78% | 0% | - |
| Nav2 MPPI, run 2 (`nav2_cmrev_r2`) | 12/18 = 67% [44, 84] | 0% | - |
| **Nav2 combined** | **26/36 = 72% [56, 84]** | 0% | - |
| Hybrid, run 1 (`hybrid_run3_esc`) | 13/18 = 72% | 25% | 22 |
| Hybrid, run 2 (`hybrid_run3_esc_r2`) | 4/6 (run A only, run B missing, see below) | 24% | 5 |
| **Hybrid combined** | **17/24 = 71% [51, 85]** | 25% | |
| Hybrid, narrow threshold 0.7 m (`hybrid_run3_esc_mc07`) | 10/18 = 56% [34, 75] | **51%** | 28 |

## Reading
- Hybrid (threshold 0.9 m) and Nav2 are level: 71% vs 72% with almost identical intervals.
- Letting DRL drive more (threshold 0.7 m) doubles the DRL share to 51% but costs success (56%): a clear trade-off.
  Failures there are again c5 and the goals right after it, plus one c4 abort.
- Nav2 run 2: c5 0/3 as always; c6 1/3 (stopped 0.16 m short twice after timeouts); one c2 timeout 10.4 m away.

## Hybrid run 2, run B missing
After run A the "return to start" goal timed out (rover at (-4.69, 2.46)) and the evaluator produced no run-B lines.
compare_protocol.sh filtered the evaluator output, so the cause was not captured. Fixed: the full output now goes to
results/<TAG>_<A|B>_evaluator_raw.log (backup made). The run-A CSV is renamed *_runB_MISSING.csv. Only run A is counted.

## Other changes
- run_drl_stack.sh / stop_drl_stack.sh: Gazebo GUI child processes (own process groups) are recorded and stopped; a GUI
  demo had left a gz server running. Tested: start/stop leaves no processes.
- GUI demos are not counted: results/*GUI_DEMO_not_comparison*.csv (Nav2 4/6, hybrid 5/6).

## Update 22:00: hybrid rerun (`hybrid_run3_esc_r3`), complete
12/18 = 67% [44, 84] (run A 5/6, run B 7/12), DRL share 25% (41.8 of 168 m), 21 escapes. Evaluator exit 0, full output in
results/hybrid_run3_esc_r3_{A,B}_evaluator_raw.log.

**Final like-for-like comparison: two complete 18-trial runs each (36 trials), same protocol:**

| | Hybrid (0.9 m) | Nav2 MPPI |
|---|---|---|
| Successes | **25/36 = 69% [53, 82]** | **26/36 = 72% [56, 84]** |
| Mean time (successes) | 74 s | 77 s |
| Per goal c1..c6 | 5/6, 5/6, 6/6, 6/6, 0/6, 3/6 | 6/6, 4/6, 6/6, 6/6, 0/6, 4/6 |
| DRL share of distance | 25% | 0% |

The incomplete repeat (`*_runB_MISSING.csv`, 4/6) is left out of this table.

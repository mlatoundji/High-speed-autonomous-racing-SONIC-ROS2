# Benchmark configs

YAML run lists for `[../benchmark.py](../benchmark.py)`.  
Output: `[results/benchmark_<timestamp>/](../../results/)` (`config.yaml`, `summary.csv`, `figures/`).

## Experiment matrix (REPORT Section 5)


| Config                                                     | Report      | Stack / line                       | Notes                                                                                                             |
| ---------------------------------------------------------- | ----------- | ---------------------------------- | ----------------------------------------------------------------------------------------------------------------- |
| —                                                          | R0 baseline | Stanley, centerline                | Frozen in `[results/baseline_2026-05-20T22-07-12/](../../results/baseline_2026-05-20T22-07-12/)`; not re-run here |
| `[smoke.yaml](smoke.yaml)`                                 | —           | Stanley, centerline, 1 lap         | Harness smoke test                                                                                                |
| `[r1_pp_vs_stanley.yaml](r1_pp_vs_stanley.yaml)`           | 5.2 R1      | Stanley vs PP, centerline, default | 3 laps, warmup 1 per run (2 effective laps in summary)                                                              |
| `[r2_tuning_sweep.yaml](r2_tuning_sweep.yaml)`             | 5.3 R2      | PP only, 3 profile labels          | **Temporary**: profile does not change yaml yet                                                                   |
| `[r3_racing_line.yaml](r3_racing_line.yaml)`               | 5.4 R3      | Stanley, racing                    | Headline: ~172 s vs centerline ~190 s                                                                             |
| `[r3_pp_racing_optional.yaml](r3_pp_racing_optional.yaml)` | 5.4 note    | PP, racing                         | Optional: integration coupling repro                                                                              |
| `[r4_latency_sweep.yaml](r4_latency_sweep.yaml)`           | 5.5 R4      | Stanley, racing                    | `latency_ms`: 0, 100, 200, 300, 500, 1000                                                                         |


Historical snapshots: `[benchmark_2026-05-23T16-28-02](../../results/benchmark_2026-05-23T16-28-02)`.

## Run order (full reproduction)

```bash
python3 scripts/benchmark.py --config scripts/configs/r1_pp_vs_stanley.yaml
python3 scripts/benchmark.py --config scripts/configs/r3_racing_line.yaml
python3 scripts/benchmark.py --config scripts/configs/r4_latency_sweep.yaml
# Placeholder batch (optional):
python3 scripts/benchmark.py --config scripts/configs/r2_tuning_sweep.yaml
# Optional coupling experiment:
python3 scripts/benchmark.py --config scripts/configs/r3_pp_racing_optional.yaml
```

Dry-run any matrix:

```bash
python3 scripts/benchmark.py --config scripts/configs/r4_latency_sweep.yaml --dry-run
```

## Schema


| Field            | Default         | Description                                        |
| ---------------- | --------------- | -------------------------------------------------- |
| `stack`          | `stanley`       | `stanley`, `mpc`, `pure_pursuit`                   |
| `profile`        | `default`       | Label in CSV only (no yaml switching unless noted) |
| `line`           | `centerline`    | `centerline` or `racing`                           |
| `latency_ms`     | `0`             | `latency_injector` delay                           |
| `odom_noise_std` | `0.0`           | `odom_noise_injector` noise                        |
| `lap_count`      | `3`             | Laps to wait for per launch                        |
| `warmup_laps`    | CLI default `1` | Laps excluded from `summary.csv` stats             |


## Expected magnitudes (sanity check)

- **R1**: Stanley ~190 s; PP ~195–196 s (centerline)
- **R3**: Stanley racing ~172 s
- **R4**: U-curve; best ~164 s @ 300 ms; 500 ms slower; 1000 ms timeout/off-track

Exact values may differ from 2026-05-23 runs (Gazebo state, BOF, etc.).
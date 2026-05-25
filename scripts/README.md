# Scripts

| File | Purpose |
|------|---------|
| [`benchmark.py`](benchmark.py) | Batch race benchmarks → `results/benchmark_<timestamp>/` |

Configs: [`configs/`](configs/) (R1–R4 matrices aligned with [`docs/pp_racing_line_2026-05-23/REPORT.md`](../docs/pp_racing_line_2026-05-23/REPORT.md)).

```bash
pip install pyyaml   # or: pip install -r requirements.txt

python3 scripts/benchmark.py --smoke
python3 scripts/benchmark.py --config scripts/configs/r1_pp_vs_stanley.yaml
python3 scripts/benchmark.py --config scripts/configs/r4_latency_sweep.yaml
python3 scripts/benchmark.py --config r4_latency_sweep.yaml --dry-run   # resolves under configs/
```

Each run writes `results/benchmark_<timestamp>/config.yaml` and `summary.csv`.

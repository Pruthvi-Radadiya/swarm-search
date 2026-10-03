# Analysis — coverage vs time

| File | Role |
|------|------|
| `coverage_logger.py` | Subscribe `/shared_map` (Transient Local), append CSV every 5 s |
| `coverage.csv` | Raw counts: unknown / free / occupied / known_frac of **full canvas** |
| `coverage_pct.png` | Demo plot: **100% = known cells at end of run** (turtle world filled) |

The merge canvas is ~20×20 m; the turtle playable area is small, so raw `known_frac` plateaus around a few percent even when the house looks “full” in RViz. Normalize by the final `free+occupied` for a 0–100% curve.

See also: root `README.md` → **Coverage vs time**, `docs/engineering-notes-map-explore.md`.

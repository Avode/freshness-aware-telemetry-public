# Evidence provenance

These are selected records from the author's September 2026 simulation runs. They are not synthetic samples or independent replications. The operational twin did not receive the simulator truth fields in `route/pose-errors.jsonl`; those fields came from a separate offline evaluator.

| Public file | Recorded source and transformation |
|---|---|
| `route/pose-errors.jsonl` | Entire evaluator file from run `autonomy-20260928-112139`; byte-identical. SHA-256 `010fd5bb6d4341156c9d0b4289367805248e94318aa07297cbd1bd83643fe4f8`. |
| `route/mission-events.jsonl` | Entire mission-event file from the same run; byte-identical. SHA-256 `d5d36e5da4f5449255578a00076c5694ebe198566b912be80218fe2c13769fec`. |
| `reports/stale-telemetry-diagnosis.json` | Original compact incident diagnostic; byte-identical. |
| `reports/telemetry-freshness-validation.json` | Original compact later-session diagnostic; byte-identical. |
| `reports/mqtt-validation.json` | Original local stand-in receiver validation; byte-identical. |
| `reports/command-center-baseline.json` | Original report with its local absolute `run` path replaced by the run ID; numeric and event fields unchanged. |
| `reports/command-center-validation.json` | Original report with its local absolute `run` path replaced by the run ID; numeric and event fields unchanged. |
| `reports/long-run-summary.json` | Curated Ubuntu-side status and event summary for run `autonomy-20260928-161113`; the source status/database files and Mac journal are not bundled. |

The earlier no-ACK incident and the later 28-second diagnostic are unmatched sessions. Two repairs and the viewing conditions changed together. Their queue charts describe observations and cannot isolate a causal effect. Gateway ACK counters count messages, including repeats. The long-run wall duration mentioned in the manuscript depends on separate journals that are not in this release.

Regenerate the three evidence plots with `python3 research/figures/build_evidence_figures.py` from the repository root. That script asserts the recorded sample count and selected summary statistics while rendering. The architecture diagram source is `research/figures/architecture.dot` and can be rendered with `dot -Tpng research/figures/architecture.dot -o research/figures/architecture.png`.

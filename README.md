# Falcon Eye

Forward-looking aircraft collision-awareness simulation prototype — LiDAR + radar sensor fusion, multi-target tracking, and slack-priority triage with a reflex fast-path, validated against 2,400 Monte Carlo trials and 8 independently-injected failure modes.

**Project stage:** Prototype (simulation-validated, not yet hardware-tested — see `outputs/Falcon_Eye_Technical_Report.docx`, Section 13, for the phased plan toward real-world validation).

This is a research simulation, not a flight-certified system. See the [Documents in this repo](#documents-in-this-repo) section below for the full technical report, pitch deck, and submission write-up.

## Requirements

- Python 3.9+
- No external services, API keys, or paid infrastructure — everything runs locally

## Setup

```bash
# from the project root
pip install -r requirements.txt
```

That installs `numpy`, `scipy`, `matplotlib`, and `pandas`. No other setup is needed — there's no build step, no environment variables, and no data to download.

## Running it

Three independent entry points, from simplest to most complete. All of them can be run directly with `python3` from the project root — no arguments needed.

### 1. Run the test suite first (fastest way to confirm everything works)

```bash
python3 tests/test_priority_hysteresis.py
```

Expect three `PASS` lines. These specifically verify the safety-critical property of the priority-triage engine: a genuine escalation or reflex trigger is never delayed, only noise-driven de-escalation is damped.

### 2. Multi-target triage demo (fastest way to see the system work)

```bash
mkdir -p outputs/figures   # only needed once
python3 demo_multi_target_scenario.py
```

Simulates three simultaneous targets approaching a host aircraft, triaged every timestep by the slack-priority/reflex engine. Prints the reflex-trigger log and the raw-vs-hysteresis-stabilized priority hand-offs to the console, and writes `outputs/figures/multi_target_triage.png` (the same figure used in the technical report and deck).

Runtime: a few seconds.

`demo_multi_target.py` (no `_scenario` suffix) is a smaller, static 3-observation version of the same triage logic, useful for reading the engine's behavior in isolation without the full time-stepped simulation.

### 3. Full validation suite (reproduces every number in the technical report)

```bash
mkdir -p outputs/figures   # only needed once
python3 run_demo.py
```

Runs, in order: the clear-air baseline scenario, the fog-progression comparison (clear → light → moderate → dense fog), all 8 failure-injection scenarios, and the full 2,400-trial Monte Carlo comparison (LiDAR-only vs. radar-only vs. fusion, across 4 fog densities, 200 trials each). Writes every figure and CSV table found in `outputs/` to that same folder, overwriting them with freshly-generated results.

**Runtime: several minutes** (the Monte Carlo comparison alone is 2,400 simulated trials). This is the script that produced every chart and table in `Falcon_Eye_Technical_Report.docx` and `Falcon_Eye_Pitch_Deck.pptx`.

## Project structure

```
falcon_eye/
    aircraft.py                  Kinematic model for host + target aircraft
    sensor.py                    Falcon Eye LiDAR-like sensor model
    radar.py                     Independent radar sensor model (for fusion)
    fog.py                       Beer-Lambert atmospheric fog/degradation model
    collision.py                 Collision-risk assessment + warning logic
    failures.py                  8-mode failure-injection framework
    tracking.py                  Alpha-beta tracker (single-target)
    multi_target_priority.py     Slack-priority triage engine + reflex fast-path
                                  + PriorityHysteresis (asymmetric hysteresis)
    multi_target_simulation.py   N-target simulation loop wiring the above together
    simulation.py                Single-target simulation loop
    monte_carlo.py                Randomized-trial experiment harness
    metrics.py                    Detection rate / false-alarm rate / TTC error etc.
    plots.py                      All figure generation

tests/
    test_priority_hysteresis.py  Safety-property unit tests

demo_multi_target_scenario.py    3-target, time-stepped triage demo
demo_multi_target.py             Static, single-cycle triage demo
run_demo.py                       Full validation suite (all report figures/tables)

outputs/                          Generated figures, CSVs, and this project's
                                   technical report + pitch deck
```

## Documents in this repo

- **`outputs/Falcon_Eye_Technical_Report.docx`** — full methodology, findings, limitations, and the phased real-world pilot plan (Section 13)
- **`outputs/Falcon_Eye_Pitch_Deck.pptx`** — presentation deck
- **`Falcon_Eye_Devpost_Submission.md`** — the project write-up in Inspiration/What it does/How we built it format
- **`Falcon_Eye_Demo_Video_Script.md`** — timed narration script for the demo video

## A note on what's simulated vs. real

Every number in this project — detection rates, false-alarm rates, warning lead times — comes from the simulation models in `falcon_eye/`, not from real sensor hardware or flight data. The sensor and fog models are physically-motivated (Beer-Lambert extinction, probability-of-detection curves informed by range and atmospheric transmittance) but not yet calibrated against real LiDAR/radar datasheets or flight-test data. That calibration is Phase 0 of the pilot plan in the technical report, and is the single most important caveat when presenting these results to anyone outside this project.

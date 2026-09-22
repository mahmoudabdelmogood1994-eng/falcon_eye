## Inspiration

Aircraft situational awareness has a quiet blind spot: it tends to fail exactly when it's needed most. Two real, well-documented crashes show the same failure pattern playing out at completely different scales:

- **January 26, 2020 — Calabasas, CA.** A helicopter carrying Kobe Bryant, his daughter Gianna, and seven others flew into thick fog under visual flight rules. NTSB investigators concluded the pilot lost outside visual reference, became spatially disoriented, and lost control of the aircraft — with no independent sensor layer on board to catch what his own eyes and instruments together didn't. **All 9 aboard were killed.**
- **May 9, 2012 — Mount Salak, Indonesia.** A Sukhoi Superjet 100 on a sales demonstration flight requested a low-altitude orbit near mountainous terrain obscured by thick cloud cover. The aircraft's terrain awareness warning system fired correctly — the crew, unable to see the terrain and unaware of its true position relative to the mountain, dismissed the warning as a system fault. **All 45 aboard were killed.** The investigation's core finding was crew reliance on visual judgment in conditions where there was none available, with no independent confirmation to override that misplaced trust.

These aren't outliers. According to the FAA, controlled flight into terrain — an airworthy aircraft flown into terrain, water, or an obstacle with the crew unaware until impact — accounts for **17% of all general aviation fatalities**, and IATA's accident data shows **more than 90% of CFIT accidents that occur are fatal**. A peer-reviewed analysis of four decades of NTSB general aviation data found weather was a cause or contributing factor in **35% of fatal accidents**, and **60% of those** happened after a pilot continued under visual flight rules into clouds or fog — a pattern still responsible for roughly **100 U.S. general aviation deaths every year**, and an estimated **$1.6-4.6 billion in annual accident costs** to the people and institutions affected.

The pattern is the same whether it's a single-engine helicopter or a commercial jet on a demo flight: visibility degrades quietly, with no explicit failure signal, and a single point of situational awareness — usually the pilot's own eyes, sometimes an instrument the crew has learned to distrust — has nothing independent backing it up when it fails. Under the FAA's new Part 108 beyond-visual-line-of-sight framework, "detect-and-avoid" is also moving from a nice-to-have to a compliance requirement for routine drone and advanced-air-mobility operations — the regulatory ground is shifting under a problem that, as these incidents and numbers show, is still actively killing people.

We asked a simple question: what would it take to build a forward-looking collision-awareness layer that's honest about its own limits — one that doesn't quietly assume a sensor still works once conditions get bad? That question became Falcon Eye.

*Sources: NTSB probable cause report, Calabasas, CA accident (Jan. 26, 2020), briefed Feb. 9, 2021; Indonesian National Transportation Safety Committee final report, Mount Salak accident, released Dec. 18, 2012; FAA (CFIT fatality share); IATA CFIT Accident Analysis Report, 2010–2014 data; Fultz & Ashley, "Fatal weather-related general aviation accidents in the United States," Physical Geography, 2016 (NTSB data, 1982–2013); Sobieralski, "The cost of general aviation accidents in the United States," Transportation Research Part A, 2013.*

## Project Stage

**Prototype.** The full sense → detect → track → triage → warn pipeline is implemented, runnable, and validated against 2,400 Monte Carlo trials plus 8 independently-injected failure modes — but entirely in simulation. No hardware sensor, real flight, or real operator has touched this system yet. We are not at MVP (that would imply a deployable product) or Demo Ready (that requires the recorded demo video and completed customer-discovery conversations currently in progress). Section 13 of our technical report lays out the concrete, phased path — bench sensor calibration, ground-vehicle surrogate testing, then a small-UAS flight test — from this stage toward Pilot Ready.

## Technology Used

- **Language:** Python 3
- **Core libraries:** NumPy, SciPy, Matplotlib (simulation, sensor/fog models, plotting)
- **Testing:** Python's built-in `assert`-based unit tests (`tests/test_priority_hysteresis.py`) verifying the safety-critical properties of the priority-hysteresis logic
- **Architecture:** a modular package (`falcon_eye/`) — kinematics, sensor models (LiDAR-like + independent radar), Beer-Lambert atmospheric fog/degradation model, collision-risk and warning logic, an 8-mode failure-injection framework, an alpha-beta tracker, a slack-priority multi-target triage engine with asymmetric-hysteresis stabilization, and a Monte Carlo experiment harness
- **Deliverable generation:** `python-docx` (technical report), `pptxgenjs` (pitch deck), standard `csv`/`markdown` for data tables and this write-up
- **No external APIs, cloud services, or paid infrastructure** — the entire prototype runs locally with open-source tooling, which is deliberate: it keeps the barrier to reproducing or extending our results at zero cost for reviewers, partners, or future contributors

## What it does

Falcon Eye is a simulation-validated, forward-looking collision-awareness system that fuses LiDAR and radar to keep working when either sensor alone would fail.

- **Fog-honest sensing.** We never assumed LiDAR "sees through" fog. We modeled Beer-Lambert atmospheric extinction and let detection probability, noise, and false-return rate actually degrade with fog density — then measured exactly how much LiDAR alone breaks down (spoiler: completely, in dense fog).
- **Sensor fusion that earns its keep.** Radar's near-fog-invariant propagation backstops LiDAR exactly where optical sensing fails, recovering full detection across every fog condition we tested.
- **Multi-target triage.** A slack-priority sort plus a reflex fast-path flags a genuinely critical target immediately, before lower-priority tracks are even scored — validated on three simultaneous tracks with zero false alarms and zero missed threats.
- **Priority-sort hysteresis.** When two targets have near-identical urgency, raw priority can flicker on measurement noise alone. We built asymmetric hysteresis that escalates attention instantly but damps meaningless de-escalation — cutting flicker by 93% without ever delaying a real alert.

## How we built it

**Simulation core (Python / NumPy / SciPy / Matplotlib):** a modular pipeline — aircraft kinematics, a configurable LiDAR-like sensor model, an independent radar model, a Beer-Lambert fog/degradation model, collision-risk and warning logic, and an 8-mode failure-injection framework (dropout, intermittent failure, biased range, false-detection bursts, added latency, data-link loss, interference, total sensor loss).

**Tracking & triage:** an alpha-beta tracker fuses per-target range/angle/velocity estimates; a slack-priority triage engine ranks every simultaneous track by urgency and fast-paths a critical one to an immediate reflex trigger; a hysteresis layer stabilizes which target holds top priority without ever delaying an escalation.

**Validation:** a Monte Carlo harness ran 2,400 randomized trials — 3 sensing modes (LiDAR-only, radar-only, fusion) × 4 fog densities × 200 trials each — scoring detection rate, missed-detection rate, false-alarm rate, warning lead time, and TTC estimation error against every trial. Every failure-injection mode was independently exercised against the same baseline scenario. Safety-critical logic (hysteresis escalation and reflex override) is locked in with dedicated unit tests, not just eyeballed from a plot.

**Documentation:** a full technical report (methodology, findings, limitations, and a phased real-world pilot plan) and a data-backed pitch deck, both built from the same numbers that came out of the simulation — nothing in either document is a rounder, friendlier version of the truth.

## Challenges we ran into

- **A tracker bug that produced a false alarm, not just a bad metric.** Our alpha-beta tracker derived closing velocity by differencing noisy range estimates over a fixed timestep, which amplifies measurement noise by 1/dt. It was quietly inflating our TTC-error metric — but it took wiring in live multi-target triage to see it cause something unambiguous: a target with a true 53-second time-to-collision spuriously triggered a critical reflex alert at t=1.1s. We traced it to the noise amplification and fixed it by low-pass filtering the sensor's own directly-measured velocity channel instead of differencing range. Confirmed stable afterward, and the false trigger did not recur.
- **Fusion isn't automatically safer.** Our first fusion implementation accepted a detection from either sensor rather than requiring agreement, which meant it actually increased false-alarm rate relative to radar alone in dense fog. We documented that honestly instead of hiding it — it's now a named limitation and a concrete next step (track-confirmation logic), not a number we quietly left out.
- **Finding the right asymmetry for hysteresis.** Naively damping priority switching risks delaying a real escalation, which is the one failure mode a collision-warning system can never have. We had to explicitly design — and then unit-test — the asymmetry: instant to escalate, damped to de-escalate, with reflex always bypassing the hold entirely.

## Accomplishments that we're proud of

- **Ran the numbers instead of asserting them:** 2,400 Monte Carlo trials, not a single demo run, back every headline claim in our deck.
- **Caught our own bug via a genuine failure, not a code review:** the tracker instability produced a visibly wrong, specific, reproducible event (a false critical alert on a low-risk target) rather than staying hidden in a soft error metric.
- **Said "this doesn't work" out loud:** LiDAR-only detection in dense fog is 0% in our results, and we led with that number instead of softening it, because it's the whole argument for why fusion matters.
- **Built a real path from simulation to hardware:** a phased pilot plan (bench sensor calibration → ground-vehicle surrogate test → small UAS flight test → degraded-visibility test) with concrete, quantitative success criteria for each phase, not just "flight test" as a roadmap bullet.

## What we learned

- Modeling a sensor honestly — including exactly how and when it fails — produces more useful results than modeling it optimistically. Our most important finding (LiDAR alone fails in fog) only exists because we refused to assume it away.
- In a safety-relevant system, the direction of an engineering decision matters as much as the decision itself. The same hysteresis pattern that stabilizes attention could just as easily have delayed a real alert if we'd gotten the asymmetry backwards — so we tested for that specifically, not just for "does it reduce flicker."
- Sensor fusion is not automatically an improvement; naive fusion can trade one failure mode (missed detections) for another (false alarms) if you don't design the agreement logic deliberately.

## What's next for Falcon Eye

- **Bench sensor calibration:** fit our simulator's detection, noise, and false-return parameters to real LiDAR/radar hardware data instead of literature-derived defaults.
- **Ground surrogate testing:** validate the full detect → track → triage → warn pipeline against real moving vehicles and real GPS ground truth, no airspace authorization required.
- **A small UAS flight test:** the critical-path step, pending a test-range or university aerospace partner — approaching a second UAS or tethered target under controlled conditions to compare real detection range and warning lead time against our simulated predictions.
- **Track-confirmation fusion:** replace our current single-hit fusion combiner with logic that requires multi-sensor agreement before declaring a track, directly addressing the false-alarm finding above.
- **Real operator validation:** conversations with UAS operators, GA pilots, and CFIs to pressure-test whether what we built matches what people actually need in the cockpit — not just what the simulation says should help.

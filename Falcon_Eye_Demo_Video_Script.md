# Falcon Eye — Demo Video Script
**Target length: 4:00–4:30 (fits the 3–5 min requirement with room to breathe)**

## How to use this
- Each block = one deck slide (or a switch to the live figure/terminal). Say the
  "SAY" text roughly as written — it's paced for natural speech, not a word-for-word
  script you must memorize.
- Practice once before recording. If you run long, cut the *italicized* optional
  lines first — the core claims still stand without them.
- Record screen + voice (OBS Studio, Zoom local recording, or QuickTime on Mac all
  work and are free). Do the deck as a full-screen slideshow (View > Present) so
  there's no browser chrome in frame.
- If you want to show the multi-target demo actually *running* rather than just
  the static figure, have a terminal window ready with
  `python3 demo_multi_target_scenario.py` pre-typed (don't run it live — the printed
  output scrolls fast; run it once beforehand and have the output visible, or just
  narrate over the deck's screenshot instead. Simpler and safer for a timed video.)

---

### [0:00–0:20] — Slide 1 (Title)
**ON SCREEN:** Title slide, radar rings animation if you have one, otherwise static.

**SAY:**
"Conventional aircraft situational awareness fails exactly when you need it most —
when visibility drops, or a single sensor goes down. This is Falcon Eye: an
independent, forward-looking collision-awareness layer built to keep working when
everything else degrades."

---

### [0:20–0:50] — Slide 2 (Problem Statement)
**ON SCREEN:** Problem statement slide — the three cards + regulatory callout.

**SAY:**
"Three things motivate this. Fog and low light quietly erode optical situational
awareness without any explicit failure signal. Any single sensor — or the
instruments reading it — can drop out or return false data. And there's no
purpose-built, always-on forward redundancy layer designed specifically for this.
*This isn't hypothetical, either — the FAA's Part 108 beyond-visual-line-of-sight
framework now requires detect-and-avoid capability for routine drone and
advanced-air-mobility operations. This category of system is becoming a
compliance requirement.*"

---

### [0:50–1:25] — Slide 3 (Solution Overview / Pipeline)
**ON SCREEN:** The Sense → Detect → Track → Triage → Warn pipeline slide.

**SAY:**
"Falcon Eye fuses LiDAR and radar returns, tracks every target in real time, and
triages multiple simultaneous threats — escalating only when a target is
genuinely on a collision course. It's built fog-honest: we don't assume LiDAR
sees through dense fog, we modeled exactly how its detection probability
degrades, and let radar's fog-invariant propagation back it up where LiDAR
alone fails."

---

### [1:25–2:05] — Slide 4 (Key Result 1 — fog chart)
**ON SCREEN:** Detection-rate-vs-fog-density bar chart.

**SAY:**
"Here's the headline result, from 2,400 randomized Monte Carlo trials. LiDAR
alone holds up fine in clear and light fog, but collapses to thirteen percent
detection in moderate fog, and zero percent in dense fog. Radar alone, and
LiDAR-plus-radar fusion, both hold one hundred percent detection across every
fog condition we tested. That's not a marginal improvement — it's the
difference between a system that works and one that silently doesn't, exactly
when visual systems are also failing."

---

### [2:05–2:45] — Slide 5 (Key Result 2 — multi-target triage)
**ON SCREEN:** Multi-target triage figure.

**SAY:**
"Real airspace has more than one target at a time. Falcon Eye triages every
tracked target by urgency and has a reflex fast-path that flags a genuinely
critical target immediately — before lower-priority tracks are even scored. In
this three-target test, the fast-closing target correctly escalates through
advisory, warning, and critical, while the slower and off-corridor traffic
correctly stay clear. *We also caught and fixed a subtle instability here: an
early version of our tracker let two similarly-urgent targets flicker for
priority fifteen times in twelve seconds. We fixed that with asymmetric
hysteresis — escalating attention is always instant, but stepping down is
damped — which cut that flicker by ninety-three percent without ever delaying
a real alert.*"

---

### [2:45–3:10] — Slide 6 (Validation rigor)
**ON SCREEN:** Big-number stats slide.

**SAY:**
"Every number in this deck traces back to a specific test run. Twenty-four
hundred Monte Carlo trials. Eight independently injected failure modes — sensor
dropout, false detections, data-link loss, and more. And under nearly all of
them, the system still produced a valid warning — evidence of graceful
degradation, not silent failure."

---

### [3:10–3:35] — Slide 7 (Business Impact)
**ON SCREEN:** Market size + regulatory + customer segment slide.

**SAY:**
"The advanced air mobility market is in the tens of billions of dollars and
growing fast, and detect-and-avoid is moving from optional to regulatory
requirement under FAA Part 108. That points to four concrete customers: UAM and
eVTOL operators, BVLOS cargo drone operators, general aviation retrofit, and
airport ground and approach operations."

---

### [3:35–3:55] — Slide 8 (Roadmap)
**ON SCREEN:** Four-phase roadmap slide.

**SAY:**
"We're at phase one: a fully validated simulation prototype — that's what
you've just seen. Next is hardware-in-the-loop testing against real sensor
datasheets, then a flight-test partnership, then certification and OEM
integration."

---

### [3:55–4:15] — Slide 9 & 10 (Team / Closing)
**ON SCREEN:** Team slide, then closing ask slide.

**SAY:**
"[Say your name(s) and one line on your background here — replace this bracket
before recording.] We're looking for mentorship from aerospace safety experts,
access to real flight-test data, and introductions to UAM and BVLOS operators.
Thanks for watching — we'd love to talk."

---

## Timing checklist before you hit record
- [ ] Replace the team-intro bracket above with your actual name(s)/background
- [ ] Fill in `[contact email]` and `[GitHub repository link]` on the deck's last
      slide *before* recording, so it's correct on screen
- [ ] Do one silent read-through with a stopwatch — if you're over 4:45, cut the
      *italicized* optional lines first
- [ ] Full-screen the deck (no browser tabs/taskbar visible)
- [ ] Check audio levels on a 10-second test clip before the real take

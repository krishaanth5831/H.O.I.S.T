# H.O.I.S.T (Hook Offset & Incremental Stem Translation)

Vinea's lowering and rehanging module. High-wire tomato plants hang from twine wound on
steel hooks along a crop wire. As a plant grows, its hook is flipped to let out twine and
hung back further along the wire, so the head stays just under the wire and the bare stem
lies along the row. H.O.I.S.T is meant to do that job: find each hook, decide how far that
plant should come down, flip the hook and move it along, without dropping the plant.

## Status: paused

Nothing physical is built. What exists is one physics probe and a design comparison
(CAD plus an animated clearance check). Work is paused until real hooks have been measured.

**Decided**
- Hooks are lifted over the wire supports (2026-10-27).
- **End effector A, the wire rider, on lift S1, the mast** (2026-10-28). The wire rider sits
  on the crop wire on a roller saddle; a normally closed twine capstan holds the plant while
  a hook rotor flips the hook and rehangs it. The mast carries a Cartesian Y/X head on its
  carriage. The other options (end effector B, the hook carrier, and the scissor and
  telescopic lifts S2/S3) were dropped.

**Pending**
- The choice of A only holds if real hooks leave enough free twine between the hook and
  the plant head for the capstan to grip: about 105 mm in the current layout (the demo and
  the CAD draw 300 mm, an unverified guess). If real hooks don't leave that much, A is out.
- Not measured yet: free twine, the hook's flip axis, twine released per half-flip,
  winding direction, hook pitch along the wire, wire height, plant weight. The values for
  these in the code are placeholders: guesses marked `unverified`, or figures taken from a
  cited source, never measurements.
- Next, once hooks are measured: code for A on S1, starting with a sim of one hook hanging
  on a wire, built on the probe below.
- No robot arm or controller is chosen. Don't read an SO-101 or a LeRobot setup into this
  repo.

## Layout

```
probes/
  hang_probe.py            can a steel hook rest on a crop wire under a full plant in MuJoCo,
                           and can a tendon act as twine that pays out? Its docstring says what
                           it showed (fat collision geoms + a stiff wire hold; payout must be ramped)
design/
  hoist_design_options.fs  Onshape FeatureScript: the greenhouse (pipe rail, crop wires, a wire
                           support, hooks, plants), end effectors A and B, lifts S1/S2/S3
  hoist_demo.py            the same geometry in MuJoCo: animates a lower-and-rehang for each
                           end effector + lift pair and runs PASS/FAIL clearance checks
runs/                      rendered videos and scratch output (gitignored)
```

## Running the scripts

Tested with Python 3.12, MuJoCo 3.8.1 and NumPy 2.2. Rendering videos also needs
`imageio`, `imageio-ffmpeg` and Pillow. Run from the repo root.

**Hang probe.** A table of collision variants, then PASS/FAIL checks on the default one.
`--windowed` opens a MuJoCo viewer with the default variant paying out on a loop.

```bash
python probes/hang_probe.py
python probes/hang_probe.py --windowed
```

**Design demo.** `--ee` is `A` or `B`, `--lift` is `S1`, `S2` or `S3`. Every run prints its
clearance checks first.

```bash
python design/hoist_demo.py --ee A --lift S1 --windowed      # watch it in a viewer window
python design/hoist_demo.py --ee A --lift S1 --checks-only   # clearance checks only, exit code 1 on a FAIL
python design/hoist_demo.py --ee A --lift S1                 # render runs/hoist_demo/hoist_demo_A_S1.mp4
```

Rendering writes over an existing video of the same name; pass `--out PATH` to keep it.
All six combinations pass their checks. The demo is a kinematic sketch, not a simulation:
every body is moved directly, and twine payout, slack and the plant drop are bookkeeping.

**Onshape.** Create a Feature Studio in an Onshape document, paste in
`design/hoist_design_options.fs`, then add the custom features to Part Studios: HOIST
Greenhouse, HOIST EE-A Wire rider, HOIST EE-B Hook carrier, and HOIST SPINE S1 Mast / S2
Scissor / S3 Telescopic. The SPINE features have switches for lift up/down, tool A or B and
showing the greenhouse. Every dimension is a first guess.

## Git

Work happens on `dev`. `main` is protected and only takes PRs from `dev`; the
`only-dev-into-main` check enforces that.

## Licence

Source-available, not open source. Copyright (c) 2026 Krishaanth Ramaraj, under the
[PolyForm Strict License 1.0.0](LICENSE): you may use the code for noncommercial purposes
as the licence defines them, but not distribute it or make changes or new works based on
it. Commercial use, modification or redistribution needs a separate written licence.

# H.O.I.S.T (Hook Offset & Incremental Stem Translation)

Vinea's lowering and rehanging module. High-wire tomato plants hang from twine wound on
steel hooks along a crop wire. As a plant grows, its hook is flipped to let out twine and
hung back further along the wire, so the head stays just under the wire and the bare stem
lies along the row. H.O.I.S.T does that job: it finds each hook, decides how far that plant
should come down, flips the hook and moves it along, without dropping the plant.

## Status

Nothing is built yet. The arm, the end effector and the lift are all still open. The first
step is a MuJoCo sim of one hook hanging on a wire under a full plant, with twine that pays
out. `probes/hang_probe.py` is the first test of that contact. The plan and the research are
in the Vinea vault, under `08 Modules/HOIST/`.

## Git

Work happens on `dev`. `main` is protected and only takes PRs from `dev`; the
`only-dev-into-main` check enforces that.

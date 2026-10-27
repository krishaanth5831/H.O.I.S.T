# CLAUDE.md — H.O.I.S.T

Vinea lowering and rehanging module: flip each plant's winding hook to let out twine, hang it
back further along the crop wire, never drop a plant. Plan and research: Vinea vault
`08 Modules/HOIST/`.

## Git workflow (hard rules)

- Commit and push straight to `dev`. **No feature branches**, not even for a PR. Never push
  to `main`. It's protected (PR required, enforce_admins on, `only-dev-into-main` required).
- Every PR is `dev` → `main`, and only when asked: `gh pr create --base main --head dev`.
  Always pass `--base` explicitly.
- **Never merge a PR**, to any branch. The user merges.

## Running heavy jobs

Sim, rendering and training runs go under a 9 GB memory cap (the user's choice, 2026-10-27):
`systemd-run --user --scope -q -p MemoryMax=9G -p MemorySwapMax=0 <cmd>`.
MuJoCo runs from the shared `lerobot` conda env until H.O.I.S.T needs its own.

## Open on purpose

The hardware isn't decided: don't assume an SO-101, a lerobot plugin, or any arm. Keep the
layout loose until the plan in the vault settles it. `probes/` holds one-off physics checks;
each one's docstring says what it showed.

## Code style

Every constant carries a comment with where its value came from: measured, copied from a
source, or a guess marked `unverified`. Use ⚠️ on traps. Code copied from B.L.U.S.H names
the B.L.U.S.H commit it came from. Every step gets a check that prints PASS/FAIL and a
`--windowed` view to watch.

Hugging Face user is `krish5831`. GitHub user is `krishaanth5831`.

# Contributing

Contributions are welcome. This is a single-maintainer project, so please open an issue
before starting anything substantial.

**Security problems do not belong in issues.** Follow [SECURITY.md](SECURITY.md).

## Branches, pull requests and releases

- `main` is the only long-lived branch and is protected: changes land through a pull
  request with the required checks green, commits on `main` are signed, and force-pushes
  are refused. Pull requests are squash-merged; GitHub deletes the branch on merge.
  The `gh-pages` branch only holds the published animation.
- Branch names: `<type>/<topic>` (`feat/`, `fix/`, `docs/`, `chore/`, `release/`);
  agents use `agent/<name>/<topic>`.
- A release is a pull request that sets the number in the `VERSION` file and moves
  `[Unreleased]` in [CHANGELOG.md](CHANGELOG.md) to a dated version. After it merges, a
  signed `vX.Y.Z` tag on that commit triggers `release.yml`, which refuses to run if the
  tag, `VERSION` and the changelog disagree. Tags are never moved or deleted. The v1.0 release
  carries the cycle video; later releases attach only the source archive.

## Running it locally

```bash
python3 -m pytest -q tests         # offline kinematics tests, no Webots needed
python3 tools/gen_world.py         # regenerate worlds/ur_scara_cell.wbt after editing lib/layout.py
webots worlds/ur_scara_cell.wbt    # needs Webots R2025a
tests/run_smoke.sh                 # headless end-to-end run in Webots, prints PASS or FAIL
```

If you change `lib/layout.py`, commit the regenerated world too: CI regenerates it and fails on any
difference. CI also runs `ruff check --select E9,F63,F7,F82` (syntax errors and undefined names
only) and cannot run the Webots smoke test, so run it yourself before a pull request that touches
the controllers or the PLC.

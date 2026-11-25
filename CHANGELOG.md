# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project uses
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [1.1.0] - 2026-11-25

Repository hygiene release; the cell itself is unchanged since 1.0. The project is a UR5e and Epson
SCARA T6 kitting cell in Webots R2025a: the SCARA fills trays with parts from an infeed conveyor, the
UR5e swaps full trays for empty ones, and a cart carries full trays out through a scanner-guarded
opening. It includes a PLC supervisor, a door, e-stop and area-scanner safety model, and an HTML5
animation published on GitHub Pages.

### Added
- MIT `LICENSE` and a `VERSION` file (1.1.0).
- CI runs the offline kinematics tests, a compile check of every Python file, and a check that the
  committed world matches `tools/gen_world.py`, on every push and pull request.
- CI, CodeQL, OpenSSF Scorecard and a tag-triggered release workflow with build provenance;
  Dependabot for actions.
- `SECURITY.md`, `CONTRIBUTING.md` and this changelog.

### Changed
- README: the cycle video link points at the v1.0 release, which carries the file, instead of
  `releases/latest`.

## [1.0] - 2026-11-25

First full cell.

[Unreleased]: https://github.com/sergiudanstan/ur-scara-cell-webots/compare/v1.1.0...HEAD
[1.1.0]: https://github.com/sergiudanstan/ur-scara-cell-webots/compare/v1.0...v1.1.0
[1.0]: https://github.com/sergiudanstan/ur-scara-cell-webots/releases/tag/v1.0

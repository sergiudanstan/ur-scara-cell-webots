# Security Policy

This repository is a Webots R2025a simulation of a robot cell: Python controllers that run inside the
Webots process (a PLC supervisor, a SCARA and a UR5e controller, an operator, and two recorders), a
world generator, and a static HTML5 animation under `site/` served from GitHub Pages. It has no server,
no accounts and no secrets, and it does not control real robots. The safety functions described in
the README (door interlock, e-stop, area scanner) are simulated; they are a model, not a certified
safety system, and must not be used to commission a real cell.

## Supported versions

| Version | Supported |
|---|---|
| 1.1.x | Current |
| 1.0 | No |

Single maintainer; only the latest release receives fixes.

## Reporting a vulnerability

**Please do not open a public issue for a security problem.** Report it privately:
**[Report a vulnerability](https://github.com/sergiudanstan/ur-scara-cell-webots/security/advisories/new)**
(Security, Advisories, Report a vulnerability).

| Stage | Target |
|---|---|
| Acknowledgement | within 3 business days |
| Triage | within 10 business days |
| Fix, or a documented decision not to fix | within 90 days |

These are best-effort targets from a single unpaid maintainer, not a contractual SLA.
There is no bug bounty; credit is offered in the advisory and changelog.

## Scope

In scope: what runs on your machine when you open the world in Webots. The controllers parse JSON
messages from the radios (`json.loads`) and read key commands from the PLC's `customData` field;
`tools/build_site.sh` and `tests/run_smoke.sh` write temporary world files with `sed` from the
checked-in world; the world file loads PROTO files from
`raw.githubusercontent.com/cyberbotics/webots/R2025a` at first run. Also in scope: the HTML5 page in
`site/`, which loads `WebotsView.js` from `cyberbotics.com/wwi/R2025a` (not pinned by hash), and
anything in this repository's workflows that could publish a release not built from `main`.

Out of scope: vulnerabilities in Webots or the Cyberbotics robot library (report to Cyberbotics), and the
accuracy of the simulation or of the safety behaviour; a wrong stop distance or cycle time is a
correctness bug, so file it as a normal issue. Webots itself cannot run in CI, so the Webots smoke test
is run by hand and is not part of the supply-chain checks below.

## Supply chain

CI runs the offline Python tests, a compile check, a check that the committed world matches its
generator, `pip-audit` on the hash-pinned CI toolchain, CodeQL (Python and Actions), and `zizmor` on the
workflows. The controllers use only the Python standard library, so there are no runtime dependencies
to audit and no SBOM is published. Actions are pinned to full commit SHAs and kept current by
Dependabot. Releases attach a source archive with Sigstore build provenance:

```
gh attestation verify ur-scara-cell-webots-v1.1.0.tar.gz --repo sergiudanstan/ur-scara-cell-webots
```

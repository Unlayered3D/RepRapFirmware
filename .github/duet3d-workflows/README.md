# Workflows in this fork

This is a fork of `Duet3D/RepRapFirmware`, so it inherited upstream's CI. Most of that CI is wired
to Duet3D's own infrastructure — their CLA repository, their package feed, their moderation policy —
and cannot work here. It was removed on 2026-09-12 rather than left to fail on every release.

## Removed, and why

| workflow | why it cannot work here |
|---|---|
| `cla.yml` | Wrote signatures into `Duet3D/CLA` using a `CLA_TOKEN` PAT this fork does not have, and read `vars.CLA_MAJOR_VERSION`/`CLA_MINOR_VERSION`, which are not set here. It gated *our* pull requests on signing *Duet3D's* contributor licence agreement, and allowlisted Duet3D's maintainers. It failed on every PR. |
| `issues.yml` | Auto-closed any issue not opened by "a Duet3D administrator", telling the reporter to post on `forum.duet3d.com` instead. Actively wrong on our own issue tracker. |
| `prerelease.yml` | Ran `gh release download -R Duet3D/RepRapFirmware` — upstream's repository, not ours — then uploaded the result to `pkg.duet3d.com` over SFTP with credentials we do not hold. Failed on every prerelease we published, including `+unlayered.4` and `+unlayered.9`. |
| `release.yml` | The same, for non-prerelease releases. |

They are not gone: `git log -- .github/workflows/` has them, and upstream still carries them if one
is ever needed as a reference.

## Kept

`deploy.yml` is left in place. It is `workflow_dispatch` only, so it never fires on its own and
costs nothing to keep. It is not currently usable as-is — it resolves each dependency repository
from `https://github.com/Duet3D/<lib>.git` rather than from our forks — but it is the closest thing
to a working CI build for this firmware and is worth adapting rather than rewriting if we ever want
release builds to happen on a runner instead of a developer's machine.

## How releases actually get published

By hand, from a developer machine. The firmware is built locally, staged in
`C:\rrf\release-staging\<version>\`, turned into a release bundle with the slicer repository's
`tools/make_firmware_bundle.py`, published with `gh release create`, and then checked end to end
with `tools/verify_firmware_updater.py --release`, which fails if the published release is less
complete than the firmware bundled with the slicer installer.

Two things that verification exists to catch, both of which have actually happened:

- A release that ships the mainboard `.uf2` without the matching IAP is a **downgrade**, because the
  slicer prefers a GitHub release over its bundled fallback and installs whatever the release omits
  as nothing at all.
- `v3.7.0-beta.1+unlayered.4` shipped the pre-2026-04-12 IAP, which cannot flash a 3.7 board, and
  its tag points at the old `3.6-dev` tip rather than at the code that shipped.

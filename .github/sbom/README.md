# SBOM

This folder contains metadata used by the NuvoISP SBOM generation workflow.

## Files

| File | Purpose |
|---|---|
| `third-party-components.yml` | Manually confirmed third-party components that Microsoft SBOM Tool may not detect automatically |
| `NuvoISP-third-party-components.md` | Human-readable third-party component note bundled into the released `SBOM_result.zip` |
| `sbom-process.md` | Operational notes for the NuvoISP SBOM workflow |

## Current manual components

The current manually added third-party components are:

- `hidapi`
- `inipp`

`BleIO.lib` and `BleIOd.lib` are intentionally excluded because they were confirmed as non-third-party components.

## How the metadata is used

The GitHub Actions workflow first builds NuvoISP and stages distributable files into:

```text
artifacts/NuvoISP
```

Then Microsoft SBOM Tool generates a base SPDX 2.2 SBOM. After that, this script runs:

```text
.github/scripts/update-sbom-manual-components.ps1
```

The script reads `.github/sbom/third-party-components.yml` and appends the listed packages and dependency relationships to:

```text
artifacts/NuvoISP/_manifest/spdx_2.2/manifest.spdx.json
```

The workflow then recalculates:

```text
manifest.spdx.json.sha256
```

and runs Microsoft SBOM Tool validation.

## Maintenance rule

When adding or changing third-party components, update `third-party-components.yml` in the same pull request.

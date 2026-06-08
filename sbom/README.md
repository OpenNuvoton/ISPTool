# SBOM

This folder contains metadata used by the NuvoISP SBOM generation workflow.

## Files

| File | Purpose |
|---|---|
| `third-party-components.yml` | Manually confirmed third-party components that Microsoft SBOM Tool may not detect automatically |

## Current manual components

The current manually added third-party components are:

- `hidapi`
- `inipp`

`BleIO.lib` and `BleIOd.lib` are intentionally excluded because they were confirmed as non-third-party components.

## How the metadata is used

The GitHub Actions workflow first runs Microsoft SBOM Tool to generate a base SPDX 2.2 SBOM. Then it runs:

```text
scripts/update-sbom-manual-components.ps1
```

The script reads `sbom/third-party-components.yml` and appends the listed packages and dependency relationships to:

```text
_manifest/spdx_2.2/manifest.spdx.json
```

After the manifest is updated, the workflow recalculates:

```text
manifest.spdx.json.sha256
```

and then validates the SBOM.

## Maintenance rule

When adding or changing third-party components, update this file and `third-party-components.yml` in the same pull request.

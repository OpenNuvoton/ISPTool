# NuvoISP SBOM Process

> Operational process notes for the NuvoISP SBOM workflow. Located under `.github/sbom/`.

This document describes the SBOM generation process for the NuvoISP project.

## Scope

The SBOM is generated for the staged NuvoISP release/drop folder:

```text
artifacts/NuvoISP
```

The workflow intentionally stages distributable files first instead of running Microsoft SBOM Tool directly on the raw MSBuild output folder. This avoids unintentionally treating build intermediates such as `.pdb`, `.lib`, and `.exp` files as release artifacts.

## Workflow

The GitHub Actions workflow is located at:

```text
.github/workflows/sbom.yml
```

The workflow performs these steps:

1. Check out the repository.
2. Verify that required SBOM files exist.
3. Build NuvoISP with MSBuild.
4. Stage distributable files into `artifacts/NuvoISP`.
5. Download Microsoft SBOM Tool.
6. Generate a base SPDX 2.2 SBOM.
7. Add manually confirmed third-party components.
8. Recalculate `manifest.spdx.json.sha256`.
9. Validate the SBOM.
10. Verify expected output files.
11. Package `SBOM_result.zip` (manifest, third-party note, validation report).
12. Package `Executable.zip` (`NuvoISP.exe`, internal runtime files, and `SBOM_result.zip`).
13. On a published release, attach `Executable.zip` to the release; on manual runs, upload it as a workflow artifact.

## Staged files

The workflow always stages:

```text
NuvoISP.exe
```

If present, it also stages these Nuvoton internal runtime files:

```text
GetChipInformation.dll
offline_isp_file_converter.exe
```

These files are not treated as third-party components, but they should be present in the SBOM build drop if they are distributed with NuvoISP.

## Manual third-party components

Some C/C++ third-party components are directly vendored into the source tree or provided as header-only files. They may not be detected as package-manager dependencies by Microsoft SBOM Tool.

Manual component metadata is maintained in:

```text
.github/sbom/third-party-components.yml
```

The workflow applies this metadata using:

```text
.github/scripts/update-sbom-manual-components.ps1
```

## Currently confirmed third-party components

| Component | License | Source location | Notes |
|---|---|---|---|
| hidapi | HIDAPI | `NuvoISP/ThirdParty/hidapi` | Vendored HIDAPI source code |
| inipp | MIT | `NuvoISP/inipp.h` | Header-only INI parser |

## Confirmed non-third-party items

| Item | Reason |
|---|---|
| `GetChipInformation.dll` | Nuvoton internal file |
| `offline_isp_file_converter.exe` | Nuvoton internal file |
| `BleIO.lib` | Confirmed non-third-party |
| `BleIOd.lib` | Confirmed non-third-party |
| NuvoISP build outputs | Project-produced artifacts |

## Workflow output artifacts

On a published release, the workflow attaches `Executable.zip` to the GitHub release.
On manual (`workflow_dispatch`) runs, the same `Executable.zip` is uploaded as a workflow artifact.

`Executable.zip` contains:

```text
NuvoISP.exe
GetChipInformation.dll
offline_isp_file_converter.exe
SBOM_result.zip
```

`SBOM_result.zip` contains:

```text
_manifest/spdx_2.2/manifest.spdx.json
_manifest/spdx_2.2/manifest.spdx.json.sha256
NuvoISP-third-party-components.md
sbom-validation-after-manual-update.json
```

## When to update this process

Update the SBOM metadata and process when:

- A new third-party component is added
- A third-party component version changes
- A third-party component license changes
- Release packaging changes
- The build output path changes
- The project starts using a package manager such as vcpkg, Conan, NuGet, or another dependency manager

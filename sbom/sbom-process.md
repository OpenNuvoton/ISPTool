# NuvoISP SBOM Process

This document describes the SBOM generation process for the NuvoISP project.

## Scope

The SBOM is generated for the NuvoISP release build output. The SBOM process covers:

- NuvoISP release artifacts produced by the build
- Automatically detected package-manager dependencies, if any
- Manually confirmed third-party components that are vendored or header-only and may not be detected automatically

## Tooling

The workflow uses Microsoft SBOM Tool to generate an SPDX 2.2 SBOM.

The GitHub Actions workflow is located at:

```text
.github/workflows/sbom.yml
```

The workflow performs these steps:

1. Check out the repository.
2. Build NuvoISP with MSBuild.
3. Download Microsoft SBOM Tool.
4. Generate a base SPDX 2.2 SBOM.
5. Add manually confirmed third-party components.
6. Recalculate `manifest.spdx.json.sha256`.
7. Validate the SBOM.
8. Upload the SBOM and validation report as workflow artifacts.

## Build drop path

The workflow currently uses:

```text
NuvoISP/Release
```

as the build drop path. This is the folder passed to Microsoft SBOM Tool with `-b`.

If the Visual Studio project output path changes, update `BUILD_DROP_PATH` in:

```text
.github/workflows/sbom.yml
```

## Build components path

The workflow currently uses:

```text
NuvoISP
```

as the build components path. This is the folder passed to Microsoft SBOM Tool with `-bc`.

## Manual third-party component handling

Some C/C++ third-party components are directly vendored into the source tree or provided as header-only files. They may not be detected as package-manager dependencies by Microsoft SBOM Tool.

Manual component metadata is maintained in:

```text
sbom/third-party-components.yml
```

The workflow applies this metadata using:

```text
scripts/update-sbom-manual-components.ps1
```

## Currently confirmed third-party components

| Component | License | Source location | Notes |
|---|---|---|---|
| hidapi | HIDAPI | `NuvoISP/ThirdParty/hidapi` | Vendored HIDAPI source code |
| inipp | MIT | `NuvoISP/inipp.h` | Header-only INI parser |

## Confirmed non-third-party items

The following items are not treated as third-party components:

| Item | Reason |
|---|---|
| `GetChipInformation.dll` | Nuvoton internal file |
| `offline_isp_file_converter.exe` | Nuvoton internal file |
| `BleIO.lib` | Confirmed non-third-party |
| `BleIOd.lib` | Confirmed non-third-party |
| NuvoISP build outputs | Project-produced artifacts |

## Output artifacts

The workflow uploads the following files:

```text
manifest.spdx.json
manifest.spdx.json.sha256
sbom-validation.json
sbom/third-party-components.yml
```

The generated SBOM is located under the build output folder:

```text
NuvoISP/Release/_manifest/spdx_2.2/manifest.spdx.json
```

## When to update this process

Update the SBOM metadata and process when:

- A new third-party component is added
- A third-party component version changes
- A third-party component license changes
- The build output path changes
- The project starts using a package manager such as vcpkg, Conan, NuGet, or another dependency manager
- The release packaging changes

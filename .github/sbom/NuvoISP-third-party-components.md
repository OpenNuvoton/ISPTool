# NuvoISP Third-Party Components — Updated

This note records the manual SBOM additions requested for NuvoISP, excluding BleIO.lib and BleIOd.lib because they were confirmed as non-third-party.

| Component | Version | License | Source location in project | SBOM handling |
|---|---:|---|---|---|
| hidapi | 0.8.0-rc1 | HIDAPI | `NuvoISP/ThirdParty/hidapi/` | Added as a package with `NuvoISP DEPENDS_ON hidapi` |
| inipp | Not asserted | MIT | `NuvoISP/inipp.h` | Added as a package with `NuvoISP DEPENDS_ON inipp` |

## Notes
- `hidapi` was recorded with `licenseDeclared = HIDAPI` and `licenseConcluded = HIDAPI`.
- `inipp` was recorded with `licenseDeclared = MIT` and `licenseConcluded = MIT`.
- `BleIO.lib` and `BleIOd.lib` were intentionally not added as third-party packages.
- Internal Nuvoton files such as `GetChipInformation.dll` and `offline_isp_file_converter.exe` are not treated as third-party components.

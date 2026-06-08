<#
.SYNOPSIS
  Adds manually confirmed third-party components to an SPDX 2.2 manifest.

.DESCRIPTION
  Microsoft SBOM Tool may not detect vendored C/C++ source code or header-only
  files as package-manager components. This script reads
  sbom/third-party-components.yml and appends manually confirmed packages and
  DEPENDS_ON relationships to the generated manifest.spdx.json.

  The YAML parser is intentionally minimal and supports only the simple
  key/value component list used by this repository.
#>

param(
  [Parameter(Mandatory = $true)]
  [ValidateNotNullOrEmpty()]
  [string] $ManifestPath,

  [Parameter(Mandatory = $true)]
  [ValidateNotNullOrEmpty()]
  [string] $ComponentsPath
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Unquote-YamlValue {
  param(
    [AllowEmptyString()]
    [string] $Value
  )

  $v = ""
  if ($null -ne $Value) {
    $v = $Value.Trim()
  }

  if ($v.Length -ge 2) {
    if (($v.StartsWith('"') -and $v.EndsWith('"')) -or
        ($v.StartsWith("'") -and $v.EndsWith("'"))) {
      return $v.Substring(1, $v.Length - 2)
    }
  }

  return $v
}

function Read-ManualComponents {
  param(
    [Parameter(Mandatory = $true)]
    [ValidateNotNullOrEmpty()]
    [string] $Path
  )

  if (-not (Test-Path $Path)) {
    throw "Components metadata file not found: $Path"
  }

  $lines = [System.IO.File]::ReadAllLines((Resolve-Path $Path))
  $components = New-Object System.Collections.Generic.List[hashtable]
  $current = $null

  foreach ($rawLine in $lines) {
    $line = $rawLine.TrimEnd()

    if ([string]::IsNullOrWhiteSpace($line)) {
      continue
    }

    if ($line.TrimStart().StartsWith("#")) {
      continue
    }

    if ($line -match '^\s*components\s*:\s*$') {
      continue
    }

    if ($line -match '^\s*-\s+name\s*:\s*(.+?)\s*$') {
      if ($null -ne $current) {
        $components.Add($current)
      }

      $current = @{}
      $current["name"] = Unquote-YamlValue $Matches[1]
      continue
    }

    if ($line -match '^\s+([A-Za-z0-9_]+)\s*:\s*(.*?)\s*$') {
      if ($null -eq $current) {
        throw "Invalid metadata format: found property before component item: $line"
      }

      $key = $Matches[1]
      $value = Unquote-YamlValue $Matches[2]
      $current[$key] = $value
      continue
    }

    throw "Unsupported metadata line: $line"
  }

  if ($null -ne $current) {
    $components.Add($current)
  }

  return @($components)
}

function Assert-RequiredField {
  param(
    [Parameter(Mandatory = $true)]
    [hashtable] $Component,

    [Parameter(Mandatory = $true)]
    [string] $Field
  )

  $componentName = "<unknown>"
  if ($Component.ContainsKey("name")) {
    $componentName = [string] $Component["name"]
  }

  if (-not $Component.ContainsKey($Field) -or
      [string]::IsNullOrWhiteSpace([string] $Component[$Field])) {
    throw "Component '$componentName' is missing required field '$Field'."
  }
}

function Add-Or-Replace-JsonProperty {
  param(
    [Parameter(Mandatory = $true)]
    [object] $Object,

    [Parameter(Mandatory = $true)]
    [string] $Name,

    [AllowNull()]
    [object] $Value
  )

  $property = $Object.PSObject.Properties[$Name]
  if ($null -eq $property) {
    $Object | Add-Member -MemberType NoteProperty -Name $Name -Value $Value
  } else {
    $Object.$Name = $Value
  }
}

if (-not (Test-Path $ManifestPath)) {
  throw "Manifest not found: $ManifestPath"
}

$manifest = Get-Content $ManifestPath -Raw -Encoding UTF8 | ConvertFrom-Json -Depth 100

if ($manifest.spdxVersion -ne "SPDX-2.2") {
  throw "Unsupported SPDX version '$($manifest.spdxVersion)'. This script expects SPDX-2.2."
}

$components = Read-ManualComponents -Path $ComponentsPath

if ($components.Count -eq 0) {
  throw "No manual components found in $ComponentsPath."
}

$packages = @()
if ($null -ne $manifest.packages) {
  $packages = @($manifest.packages)
}

$relationships = @()
if ($null -ne $manifest.relationships) {
  $relationships = @($manifest.relationships)
}

$rootPackageId = "SPDXRef-RootPackage"
$requiredFields = @(
  "name",
  "spdxId",
  "supplier",
  "licenseDeclared",
  "licenseConcluded",
  "copyrightText",
  "downloadLocation",
  "relationship",
  "relationshipFrom"
)

foreach ($component in $components) {
  foreach ($field in $requiredFields) {
    Assert-RequiredField -Component $component -Field $field
  }

  $spdxId = [string] $component["spdxId"]

  $existingPackage = $packages | Where-Object { $_.SPDXID -eq $spdxId } | Select-Object -First 1

  if ($null -eq $existingPackage) {
    $package = [ordered] @{
      name = [string] $component["name"]
      SPDXID = $spdxId
      downloadLocation = [string] $component["downloadLocation"]
      filesAnalyzed = $false
      licenseConcluded = [string] $component["licenseConcluded"]
      licenseDeclared = [string] $component["licenseDeclared"]
      copyrightText = [string] $component["copyrightText"]
      supplier = [string] $component["supplier"]
    }

    if ($component.ContainsKey("version") -and
        -not [string]::IsNullOrWhiteSpace([string] $component["version"]) -and
        [string] $component["version"] -ne "NOASSERTION") {
      $package["versionInfo"] = [string] $component["version"]
    }

    $packages += [pscustomobject] $package
    Write-Host "Added package: $spdxId"
  } else {
    Write-Host "Package already exists, skipping: $spdxId"
  }

  $relationshipFrom = [string] $component["relationshipFrom"]
  $relationshipType = [string] $component["relationship"]

  if ($relationshipFrom -ne $rootPackageId) {
    throw "Unsupported relationshipFrom '$relationshipFrom'. Expected '$rootPackageId'."
  }

  $existingRelationship = $relationships |
    Where-Object {
      $_.spdxElementId -eq $relationshipFrom -and
      $_.relationshipType -eq $relationshipType -and
      $_.relatedSpdxElement -eq $spdxId
    } |
    Select-Object -First 1

  if ($null -eq $existingRelationship) {
    $relationships += [pscustomobject] ([ordered] @{
      relationshipType = $relationshipType
      relatedSpdxElement = $spdxId
      spdxElementId = $relationshipFrom
    })

    Write-Host "Added relationship: $relationshipFrom $relationshipType $spdxId"
  } else {
    Write-Host "Relationship already exists, skipping: $relationshipFrom $relationshipType $spdxId"
  }
}

Add-Or-Replace-JsonProperty -Object $manifest -Name "packages" -Value @($packages)
Add-Or-Replace-JsonProperty -Object $manifest -Name "relationships" -Value @($relationships)

# Basic structural checks before writing.
$allIds = New-Object System.Collections.Generic.HashSet[string]
[void] $allIds.Add([string] $manifest.SPDXID)

foreach ($pkg in @($manifest.packages)) {
  if ([string]::IsNullOrWhiteSpace([string] $pkg.SPDXID)) {
    throw "A package is missing SPDXID."
  }

  if (-not $allIds.Add([string] $pkg.SPDXID)) {
    throw "Duplicate SPDXID found: $($pkg.SPDXID)"
  }
}

if ($null -ne $manifest.files) {
  foreach ($file in @($manifest.files)) {
    if ([string]::IsNullOrWhiteSpace([string] $file.SPDXID)) {
      throw "A file is missing SPDXID."
    }

    if (-not $allIds.Add([string] $file.SPDXID)) {
      throw "Duplicate SPDXID found: $($file.SPDXID)"
    }
  }
}

foreach ($rel in @($manifest.relationships)) {
  if (-not $allIds.Contains([string] $rel.spdxElementId)) {
    throw "Relationship spdxElementId does not exist: $($rel.spdxElementId)"
  }

  if (-not $allIds.Contains([string] $rel.relatedSpdxElement)) {
    throw "Relationship relatedSpdxElement does not exist: $($rel.relatedSpdxElement)"
  }
}

$manifest |
  ConvertTo-Json -Depth 100 |
  Set-Content $ManifestPath -Encoding utf8

Write-Host "Updated manifest: $ManifestPath"

param(
  [Parameter(Mandatory = $true)]
  [string] $ManifestPath,

  [Parameter(Mandatory = $true)]
  [string] $ComponentsPath
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Unquote-YamlValue {
  param([AllowEmptyString()][string] $Value)

  $v = $Value.Trim()

  if ($v.Length -ge 2) {
    if (($v.StartsWith('"') -and $v.EndsWith('"')) -or
        ($v.StartsWith("'") -and $v.EndsWith("'"))) {
      return $v.Substring(1, $v.Length - 2)
    }
  }

  return $v
}

function Convert-SimpleYamlComponents {
  param([Parameter(Mandatory = $true)][string[]] $Lines)

  $components = New-Object System.Collections.Generic.List[hashtable]
  $current = $null

  foreach ($rawLine in $Lines) {
    $line = $rawLine.TrimEnd()

    if ([string]::IsNullOrWhiteSpace($line)) { continue }
    if ($line.TrimStart().StartsWith("#")) { continue }
    if ($line -match '^\s*components\s*:\s*$') { continue }

    if ($line -match '^\s*-\s+name\s*:\s*(.+?)\s*$') {
      if ($null -ne $current) { $components.Add($current) }
      $current = @{}
      $current["name"] = (Unquote-YamlValue $Matches[1])
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
  }

  if ($null -ne $current) { $components.Add($current) }
  return $components
}

function Assert-RequiredField {
  param([hashtable] $Component, [string] $Field)

  if (-not $Component.ContainsKey($Field) -or
      [string]::IsNullOrWhiteSpace([string] $Component[$Field])) {
    throw "Component '$($Component["name"])' is missing required field '$Field'."
  }
}

if (-not (Test-Path $ManifestPath)) {
  throw "Manifest not found: $ManifestPath"
}

if (-not (Test-Path $ComponentsPath)) {
  throw "Components metadata file not found: $ComponentsPath"
}

$manifest = Get-Content $ManifestPath -Raw -Encoding UTF8 | ConvertFrom-Json -Depth 100

if ($manifest.spdxVersion -ne "SPDX-2.2") {
  throw "Unsupported SPDX version '$($manifest.spdxVersion)'. This script expects SPDX-2.2."
}

$components = Convert-SimpleYamlComponents -Lines (Get-Content $ComponentsPath -Encoding UTF8)

if ($components.Count -eq 0) {
  throw "No manual components found in $ComponentsPath."
}

if ($null -eq $manifest.packages) {
  $manifest | Add-Member -MemberType NoteProperty -Name packages -Value @()
}

if ($null -eq $manifest.relationships) {
  $manifest | Add-Member -MemberType NoteProperty -Name relationships -Value @()
}

$rootPackageId = "SPDXRef-RootPackage"
$existingPackageIds = @{}

foreach ($pkg in $manifest.packages) {
  $existingPackageIds[$pkg.SPDXID] = $true
}

foreach ($component in $components) {
  foreach ($field in @(
    "name",
    "spdxId",
    "supplier",
    "licenseDeclared",
    "licenseConcluded",
    "copyrightText",
    "downloadLocation",
    "relationship",
    "relationshipFrom"
  )) {
    Assert-RequiredField -Component $component -Field $field
  }

  $spdxId = [string] $component["spdxId"]

  if (-not $existingPackageIds.ContainsKey($spdxId)) {
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

    $manifest.packages += [pscustomobject] $package
    $existingPackageIds[$spdxId] = $true
    Write-Host "Added package: $spdxId"
  } else {
    Write-Host "Package already exists, skipping: $spdxId"
  }

  $relationshipFrom = [string] $component["relationshipFrom"]
  $relationshipType = [string] $component["relationship"]

  if ($relationshipFrom -ne $rootPackageId) {
    throw "Unsupported relationshipFrom '$relationshipFrom'. Expected '$rootPackageId'."
  }

  $exists = $false
  foreach ($rel in $manifest.relationships) {
    if ($rel.spdxElementId -eq $relationshipFrom -and
        $rel.relationshipType -eq $relationshipType -and
        $rel.relatedSpdxElement -eq $spdxId) {
      $exists = $true
      break
    }
  }

  if (-not $exists) {
    $manifest.relationships += [pscustomobject] ([ordered] @{
      relationshipType = $relationshipType
      relatedSpdxElement = $spdxId
      spdxElementId = $relationshipFrom
    })

    Write-Host "Added relationship: $relationshipFrom $relationshipType $spdxId"
  } else {
    Write-Host "Relationship already exists, skipping: $relationshipFrom $relationshipType $spdxId"
  }
}

$allIds = New-Object System.Collections.Generic.HashSet[string]
[void] $allIds.Add([string] $manifest.SPDXID)

foreach ($pkg in $manifest.packages) {
  if (-not $allIds.Add([string] $pkg.SPDXID)) {
    throw "Duplicate SPDXID found: $($pkg.SPDXID)"
  }
}

if ($null -ne $manifest.files) {
  foreach ($file in $manifest.files) {
    if (-not $allIds.Add([string] $file.SPDXID)) {
      throw "Duplicate SPDXID found: $($file.SPDXID)"
    }
  }
}

foreach ($rel in $manifest.relationships) {
  if (-not $allIds.Contains([string] $rel.spdxElementId)) {
    throw "Relationship spdxElementId does not exist: $($rel.spdxElementId)"
  }

  if (-not $allIds.Contains([string] $rel.relatedSpdxElement)) {
    throw "Relationship relatedSpdxElement does not exist: $($rel.relatedSpdxElement)"
  }
}

$manifest |
  ConvertTo-Json -Depth 100 |
  Set-Content $ManifestPath -Encoding UTF8

Write-Host "Updated manifest: $ManifestPath"

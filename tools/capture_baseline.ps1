param([string]$Revision = 'HEAD')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$destination = Join-Path $projectRoot 'artifacts/baseline'
New-Item -ItemType Directory -Force -Path $destination | Out-Null
$commit = (& git -c "safe.directory=$projectRoot" -C $projectRoot rev-parse $Revision).Trim()
if ($LASTEXITCODE -ne 0) { throw 'Invalid baseline revision' }
$files = @{
    'brain_node.py' = 'ros2_ws/src/so101_brain/so101_brain/brain_node.py'
    'driver_node.py' = 'ros2_ws/src/so101_driver/so101_driver/driver_node.py'
    'perception_node.py' = 'ros2_ws/src/so101_perception/so101_perception/perception_node.py'
    'sim.py' = 'sim/so101_sim/__init__.py'
    'so101.urdf' = 'ros2_ws/src/so101_description/urdf/so101.urdf'
    'so101_tek5.xml' = 'sim/so101_sim/assets/so101/so101_tek5.xml'
    'scene_tek5.xml' = 'sim/so101_sim/assets/so101/scene_tek5.xml'
}
foreach ($entry in $files.GetEnumerator()) {
    $content = & git -c "safe.directory=$projectRoot" -C $projectRoot show "${commit}:$($entry.Value)"
    if ($LASTEXITCODE -ne 0) { throw "Cannot read baseline $($entry.Value)" }
    [System.IO.File]::WriteAllText((Join-Path $destination $entry.Key), (($content -join "`n") + "`n"), [System.Text.UTF8Encoding]::new($false))
}
Set-Content -LiteralPath (Join-Path $destination 'commit.txt') -Value $commit
Get-ChildItem -LiteralPath $destination -File | Get-FileHash -Algorithm SHA256 | Select-Object Path,Hash | ConvertTo-Json | Set-Content (Join-Path $destination 'hashes.json')
Write-Output "Captured immutable baseline $commit"

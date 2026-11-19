param(
    [ValidateRange(0,1000000)][int]$Graine = 0,
    [ValidateRange(1,6)][int]$Cubes = 6,
    [switch]$Video
)
$ErrorActionPreference = 'Stop'
$racineProjet = Split-Path -Parent $PSScriptRoot
Push-Location -LiteralPath $racineProjet
try {
    docker info --format '{{.ServerVersion}}'
    if ($LASTEXITCODE -ne 0) { throw 'Ouvre Docker Desktop avant de lancer le test.' }
    $optionsPython = @('--seed',"$Graine",'--cubes',"$Cubes")
    if ($Video) { $optionsPython += '--video' }
    docker compose -f bonus/compose.classic.yml run --rm driver python3 /project/bonus/classic/calibrated_test.py @optionsPython
    if ($LASTEXITCODE -ne 0) { throw 'Essai non reussi. Le rapport est conserve dans bonus/classic/results.' }
    Write-Host 'Essai reussi selon le controle physique. Consulter le rapport pour les mesures.'
}
finally { Pop-Location }

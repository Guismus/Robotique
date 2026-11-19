param([switch]$Reconstruire)
$ErrorActionPreference = 'Stop'
$racineProjet = Split-Path -Parent $PSScriptRoot
Push-Location -LiteralPath $racineProjet
try {
    docker info --format '{{.ServerVersion}}'
    if ($LASTEXITCODE -ne 0) { throw 'Ouvre Docker Desktop puis relance ce programme.' }
    if ($Reconstruire) {
        docker compose -f bonus/compose.yml build driver
        if ($LASTEXITCODE -ne 0) { throw 'La preparation du logiciel a echoue.' }
    }
    Write-Host 'Entrainement de la vision uniquement. Les mouvements utilisent les calculs FK/IK classiques.'
    docker compose -f bonus/compose.yml run --rm driver python3 /project/bonus/train.py
    if ($LASTEXITCODE -ne 0) { throw 'L entrainement de la vision a echoue.' }
    Write-Host 'Poids : bonus/cube_net.pt ; mesures : bonus/validation_ia.json.'
}
finally { Pop-Location }

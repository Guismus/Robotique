param([switch]$SansRviz,[switch]$Demarrer,[switch]$Reset)
$ErrorActionPreference='Stop'
$racineProjet=Split-Path -Parent $PSScriptRoot
Push-Location -LiteralPath $racineProjet
try {
    $argumentsCompose=@('-f','bonus/compose.classic.yml')
    if (-not $SansRviz) { $argumentsCompose+=@('--profile','interface') }
    $argumentsLancement=@('up','-d')
    if ($Reset) { $argumentsLancement+='--force-recreate' }
    docker compose @argumentsCompose @argumentsLancement
    if ($LASTEXITCODE -ne 0) { throw 'Le lancement ROS a echoue.' }
    if ($Demarrer) {
        docker compose -f bonus/compose.classic.yml exec -T brain bash -lc 'source /opt/ros/jazzy/setup.bash && python3 /project/bonus/classic/start_ros.py'
        if ($LASTEXITCODE -ne 0) { throw 'La demande de demarrage a echoue.' }
    }
    Write-Host 'Pour lancer le mouvement : .\bonus\lancer-classique.ps1 -Demarrer'
    Write-Host 'Pour remettre les cubes au depart : .\bonus\lancer-classique.ps1 -Reset'
}
finally { Pop-Location }

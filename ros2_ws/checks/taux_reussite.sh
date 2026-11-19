#!/usr/bin/env bash
# Mesure le taux de reussite du pick & place sur plusieurs scenes (graines) de la simulation.
# Pour chaque graine : relance la sim, attend la calibration ArUco, lance /brain/start,
# puis note le resultat final du cerveau (CYCLE_REUSSI ou ECHEC_..., avec le numero d'essai).
#
# Usage, depuis le dossier docker :   bash ../ros2_ws/checks/taux_reussite.sh 1 20
# Resultat : ros2_ws/checks/taux_reussite.txt
set -u
FIRST=${1:-1}
LAST=${2:-20}
OUT=../ros2_ws/checks/taux_reussite.txt
echo "Taux de reussite, graines $FIRST a $LAST ($(date '+%Y-%m-%d %H:%M'))" > "$OUT"

ok=0
total=0
for seed in $(seq "$FIRST" "$LAST"); do
  SIM_SEED=$seed docker compose up -d --force-recreate control perception > /dev/null 2>&1
  # Attendre le driver (la construction colcon prend ~20 s), puis la calibration de la camera.
  for _ in $(seq 1 60); do docker compose logs control 2>/dev/null | grep -q "Driver node ready" && break; sleep 3; done
  docker compose restart perception > /dev/null 2>&1
  for _ in $(seq 1 30); do docker compose logs perception 2>/dev/null | grep -q "Calibration ArUco" && break; sleep 2; done
  sleep 3
  docker compose exec -T control bash -lc \
    "source /opt/ros/jazzy/setup.bash && ros2 service call /brain/start std_srvs/srv/Trigger" > /dev/null 2>&1
  # Attendre la fin : 3 essais maximum, ~40 s chacun.
  result="PAS_DE_FIN"
  for _ in $(seq 1 75); do
    line=$(docker compose logs control 2>&1 | grep -oE "(CYCLE_REUSSI|ECHEC_[A-Z_]+|ERREUR_[A-Z]+).*" | tail -1)
    if [ -n "$line" ]; then result=$line; break; fi
    sleep 2
  done
  retries=$(docker compose logs control 2>&1 | grep -oE "(SAISIE_RATEE|PAS_DANS_LE_BAC)" | tr '\n' ' ')
  total=$((total + 1))
  case "$result" in CYCLE_REUSSI*) ok=$((ok + 1)) ;; esac
  echo "graine $seed : $result ${retries:+(reprises : $retries)}" | tee -a "$OUT"
done
echo "TOTAL : $ok / $total cycles reussis" | tee -a "$OUT"

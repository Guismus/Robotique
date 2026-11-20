#!/usr/bin/env bash
# Rejoue la pile dans MuJoCo (sans ROS) sur plusieurs graines et donne le resultat physique de chacune.
# Usage :  bash ros2_ws/checks/stack_probe.sh 1 6 [hauteur_prise] [lacher]
cd "$(dirname "$0")/../../docker" || exit 1
for s in $(seq "${1:-1}" "${2:-6}"); do
  docker run --rm -e PYTHONPATH=/opt/so101/sim:/ros2_ws/src/so101_brain -e MUJOCO_GL=osmesa \
    -v "$PWD/../ros2_ws:/ros2_ws" -v "$PWD/../sim:/opt/so101/sim" tek5/control \
    python3 /ros2_ws/checks/stack_probe.py "$s" "${3:-0.025}" "${4:-0.01}" 2>&1 | grep -E "RESULTAT|Error"
done

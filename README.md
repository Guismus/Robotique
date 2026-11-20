# Tek5 — SO-ARM101 : pick & place et pile de 3 cubes (vrai bras)

Bras SO-ARM101 piloté par ROS 2 (Jazzy) dans Docker, en simulation MuJoCo ou sur le vrai bras.
Une caméra fixe au-dessus de la table repère les cubes ; le cerveau calcule les angles (IK) et vérifie
chaque étape avec la caméra (boucle fermée).

- **Phase 1** (`MODE=pick`, par défaut) : prendre le cube rouge et le déposer dans le bac.
- **Pile de 3 cubes** (`MODE=stack`) : empiler 3 cubes rouges sur un socle marron. Version simple prévue
  pour le vrai bras (on n'a que 3 cubes). Ce n'est pas la phase 2.

Sujet : [`docs/SUJET.md`](docs/SUJET.md). Guide pas à pas : [`docs/GUIDE_DEBUTANT.md`](docs/GUIDE_DEBUTANT.md).
Compte rendu de la première version : [`docs/COMPTE_RENDU.md`](docs/COMPTE_RENDU.md).
**La phase 2 du projet** (pyramide de 6 cubes, vision PyTorch) est sur la branche `bonus` (voir `bonus/README.md`).

## Résultats en simulation

| Phase | Mesure | Résultat | Fichier |
|---|---|---|---|
| 1 : cube → bac | 20 scènes (graines 1 à 20) | **20 / 20** (19 du premier coup, 1 au 2e essai) | `ros2_ws/checks/taux_reussite.txt` |
| Pile de 3 (ROS complet) | 5 scènes (graines 1 à 5) | **3 / 5** piles complètes ; les 2 échecs sont détectés (ECHEC_PILE_ABIMEE) | `ros2_ws/checks/taux_reussite_stack.txt` |
| Pile de 3 (sans ROS, vérité terrain) | 10 scènes | **7 / 10** ; 1er et 2e étage 10 / 10, le 3e est posé avec la pince inclinée de 13° | `ros2_ws/checks/stack_probe.sh` |
| Calibration caméra ArUco | 4 marqueurs, 16 coins | écart 2,1 mm avec la vraie pose, reprojection 0,56 px | log de la perception |
| Fréquences | `ros2 topic hz` | caméra et overlay 10 Hz, `/joint_states` ~23 Hz (30 visé, CPU chargé par le rendu) | — |

## Organisation

```
docker/                     conteneurs : control (driver + cerveau), perception, viz (TF du robot)
  docker-compose.yml        MODE=pick|stack, SIM_SEED=N (scène), USE_SIM=true|false
  control_entrypoint.sh     build colcon puis lance driver + cerveau avec les bons paramètres
sim/so101_sim/              simulation MuJoCo (même interface que le vrai bras)
  assets/so101/scene_tek5.xml   phase 1 : 1 cube, le bac, 4 marqueurs ArUco
  assets/so101/scene_stack.xml  pile : 3 cubes, le socle, 4 marqueurs ArUco
ros2_ws/src/
  so101_driver/             seul nœud qui parle au bras (sim ou réel) : /joint_command -> /joint_states, caméra
  so101_perception/         caméra -> positions 3D des cubes, du bac/socle ; calibration ArUco ; overlay
  so101_brain/              FK/IK (kinematics.py) et machine à états (brain_node.py)
  so101_description/        URDF + robot_state_publisher (TF)
ros2_ws/checks/             scripts de mesure et diagnostics (taux de réussite, stack_probe, ArUco)
ros2_ws/so101.rviz          vue RViz : robot, TF, image overlay, cubes détectés, pince (FK)
```

### Topics principaux

| Topic | Type | Rôle |
|---|---|---|
| `/joint_command` → `/joint_states` | JointState | commande et mesure des 6 moteurs |
| `/external_cam/image_raw` | Image | caméra, QoS *sensor data* (BEST_EFFORT) : une image perdue n'est pas renvoyée |
| `/object_position`, `/drop_box_position` | PoseStamped | cube (le plus gros) et bac/socle, repère `base_link` |
| `/objects_positions` | PoseArray | tous les cubes vus (pile) |
| `/perception/overlay`, `/perception/markers` | Image, MarkerArray | détections dessinées sur l'image, cubes en 3D dans RViz |
| `/end_effector_pose` | PoseStamped | position de la pince calculée par la FK |
| `/brain_state` | String | étape en cours (APPROCHE, SAISIE, ..., CYCLE_REUSSI, PILE_REUSSIE, ECHEC_...) |
| `/brain/start`, `/brain/stop` | Trigger | démarrer / arrêter |

## Comment ça marche

1. **Perception** : au démarrage, les 4 marqueurs ArUco collés sur la table donnent la pose de la caméra
   (`solvePnP`). Ensuite chaque tache rouge (HSV) est projetée sur le plan des cubes (z = 2,5 cm).
2. **IK** : `kinematics.py` lit l'URDF et cherche les 5 angles par moindres carrés (scipy), pince verticale.
   Pour la pile, au-delà de 12 cm de haut la pince a le droit de s'incliner (jusqu'à ~35°).
3. **Cerveau** : une étape à la fois (APPROCHE, DESCENTE, SAISIE, LEVAGE, TRANSPORT, DEPOT, RETRAIT),
   commandes lissées (0,025 rad par pas de 50 ms).
4. **Boucle fermée** : la caméra vérifie que le cube a quitté sa place après le transport, puis qu'il est
   dans le bac (ou sur la pile). Sinon le cycle est relancé depuis la nouvelle position, 3 essais maximum.
   Si le bras cache le cube, il tourne de 90° (DEGAGEMENT) pour laisser la caméra regarder.
5. **Pile de 3 cubes** : le bras prend le cube libre le plus proche du socle, le porte au-dessus de la pile
   et le lâche à la hauteur de son étage (2,7 / 7,7 / 12,7 cm). Le décalage du cube dans la pince
   (18 mm, mâchoires asymétriques) est compensé. Après chaque dépôt, le bras se dégage et la caméra
   compte les cubes restants et vérifie que le sommet de la pile est là où il doit apparaître.

## Lancer (terminal Ubuntu / WSL, depuis `docker/`)

```bash
xhost +local:docker
# Phase 1 (scène n°5)
SIM_SEED=5 docker compose up -d --force-recreate control perception viz
# Pile de 3 cubes
MODE=stack SIM_SEED=5 docker compose up -d --force-recreate control perception viz

# Quand la perception affiche "Calibration ArUco" (docker compose logs -f perception) :
docker compose exec control bash -lc "source /opt/ros/jazzy/setup.bash && ros2 service call /brain/start std_srvs/srv/Trigger"
docker compose logs -f control | grep brain     # suivre les étapes
```

RViz : `docker compose exec control bash -lc "source /ros2_ws/install/setup.bash && rviz2 -d /ros2_ws/so101.rviz"`.

Mesurer le taux de réussite (relance la sim pour chaque graine) :

```bash
bash ../ros2_ws/checks/taux_reussite.sh 1 20             # phase 1
MODE=stack bash ../ros2_ws/checks/taux_reussite.sh 1 10  # pile de 3 cubes
bash ../ros2_ws/checks/stack_probe.sh 1 10 0.025 0.0     # pile sans ROS, vérité terrain de la sim
```

## Vrai bras

Dans `docker/.env` : `USE_SIM=false`, `REAL_ROBOT_PORT=/dev/ttyACM0`, `CALIBRATION_FILE=<fichier lerobot>`.
Il faut aussi une caméra publiant `/external_cam/image_raw` et `/external_cam/camera_info`, les 4 marqueurs
ArUco imprimés (carré noir de 10 cm, positions dans `perception_node.py`) et, pour la pile, une feuille
marron/orange de 10 cm comme socle. Aucun essai sur le vrai bras n'a encore été fait.

## Limites connues

- L'orientation du cube (rotation autour de z) n'est pas utilisée : la pince le prend toujours dans le même axe.
- Pile : le bras ne monte pas au-dessus de 12 cm avec la pince verticale ; le 3e cube est posé incliné
  de 13° et tombe parfois. Un cube tombé contre la pile ne peut pas être repris (ECHEC_PILE_ABIMEE).
- Les seuils de couleur HSV sont réglés sur la simulation ; à recalibrer sous l'éclairage réel.

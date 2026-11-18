# Guide debutant : mettre le cube dans le bac

## 1. Comprendre les trois programmes

Un « node » ROS 2 est un petit programme. Un « topic » est un canal de messages.

- `driver_node.py` est le conducteur : il transmet les ordres aux moteurs et envoie les images.
- `perception_node.py` est les yeux : il cherche le rouge du cube et le marron du bac.
- `brain_node.py` organise les etapes et demande les angles des moteurs.

Le calcul « IK » transforme une position souhaitee en cinq angles. Le modele URDF contient les dimensions du bras. Les distances sont en metres, les angles en radians. La commande de pince est en pourcentage : 0 ferme, 100 ouvre. Dans les mesures `/joint_states`, la pince est convertie en radians pour RViz.

## 2. Demarrer la simulation

Prerequis : Docker avec son moteur Linux demarre. Les PDF utilisent Linux ou une VM Linux. Sur Windows, Docker Desktop doit etre lance et configure pour les conteneurs Linux. RViz demande en plus un affichage graphique Linux ; il n'est pas necessaire pour lancer les programmes.

Depuis le dossier du projet :

```sh
cd docker
docker compose up --build -d control perception
docker compose logs -f control perception
```

La construction peut prendre plusieurs minutes. Attendre la fin de `colcon build`. Garder `USE_SIM=true` (valeur par defaut). Le bras ne commence pas le cycle tout seul.

Dans un autre terminal, toujours dans `docker` :

```sh
docker compose exec control bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 topic hz /joint_states
```

Utiliser Ctrl+C pour terminer cette mesure. On attend environ 30 messages par seconde. Verifier ensuite :

```sh
ros2 topic echo /object_position --once
ros2 topic echo /drop_box_position --once
ros2 service call /brain/start std_srvs/srv/Trigger '{}'
ros2 topic echo /brain_state
```

Le bras approche, descend, ferme la pince, remonte, se deplace au-dessus du bac, ouvre la pince et se retire. Le message `CYCLE_TERMINE_A_VERIFIER` signifie que les mouvements sont termines : il ne prouve pas que le cube a ete saisi.

Pour interrompre un mouvement dans un autre terminal du conteneur :

```sh
ros2 service call /brain/stop std_srvs/srv/Trigger '{}'
```

Pour tout arreter depuis le terminal de depart : `docker compose down`.

## 3. Voir et verifier

Sur Linux avec X11, autoriser l'affichage selon les instructions du bootstrap, puis lancer `docker compose up -d viz`. Dans le conteneur control, lancer `rviz2 -d /ros2_ws/so101.rviz`. Utiliser `base_link` comme repere fixe. L'image `/external_cam/image_raw` montre la vraie scene simulee ; `/perception/mask` montre les pixels rouges retenus. RViz seul affiche le modele du bras : ce n'est pas une preuve de prise du cube.

Faire plusieurs essais avec des positions differentes. Observer que le cube quitte la table pendant le levage puis reste dans le bac apres ouverture. Noter le nombre d'essais et de reussites. Ne pas annoncer un taux de reussite avant ces essais.

## 4. Si cela ne marche pas

- Pas de connexion Docker : demarrer le moteur Docker Linux.
- Pas de position : verifier les images, les couleurs et la calibration de camera.
- Cible inaccessible : le calcul refuse de bouger ; rapprocher la cible dans la scene et relancer.
- `ERREUR_DELAI` : un moteur n'a pas atteint la cible en 15 secondes.
- Le bras bouge mais manque le cube : verifier la precision de perception, la hauteur de prise et l'orientation de la pince.

## 5. Ce qui reste avant une demonstration fiable

Cette premiere version suppose que le cube repose sur la table et utilise une hauteur de centre de 2,5 cm. Le centre du contour visible n'est pas exactement le centre du cube en perspective. Les seuils de couleur et les hauteurs doivent etre verifies dans les images.

La pose camera fournie par MuJoCo sert au demarrage. Le sujet demande ensuite votre propre calibration. Il faut mesurer des points connus ou utiliser un marqueur et estimer la pose camera. Le bras reel necessite aussi une camera calibree publiant les memes messages ; elle n'est pas configuree ici. Le blocage du couple deja present dans le driver reel est conserve.

Le programme memorise le cube et le bac au depart. Il ne suit pas un cube deplace pendant le cycle, ne planifie pas l'evitement d'obstacles et ne confirme pas automatiquement la prise. La pince du cube creux peut necessiter un ajustement de l'orientation. Tester ces points en simulation avant de passer au materiel.

## Essai valide le 7 novembre 2026

Un cycle complet avec camera a ete execute avec succes ; voir le compte rendu et son image finale. La hauteur de transport est maintenant de 8 cm et la camera regarde la table par-dessus. Le cube reste creux dans la physique.

Pour remettre le cube a sa position initiale, depuis `docker` :

```sh
docker compose restart control
```

Attendre que le driver indique qu'il est pret, puis appeler `/brain/start` comme plus haut. Si le service demande d'attendre les positions recentes, patienter quelques secondes et reessayer. Ne pas redemarrer un cycle sur le cube deja dans le bac.

Tests numeriques : `docker compose exec control python3 /ros2_ws/checks/test_kinematics.py`.

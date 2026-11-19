# Les fichiers utiles pour presenter le bonus

1. `../network.py` et `../train.py` : le petit reseau apprend les couleurs dans nos images de simulation.
2. `vision.py` : le reseau produit un masque de pixels. `shape_fit.py` ajuste la forme connue du cube sur ce contour pour calculer sa position.
3. `kinematics.py` : FK (angles vers position) et IK (position vers angles). Ce sont des calculs, pas de l IA.
4. `calibrated_planning.py` : les six destinations, l ordre des mouvements et les passages au-dessus des cubes deja poses.
5. `driver_node.py`, `vision_node.py`, `brain_node.py` : les trois programmes ROS. Le cerveau transmet les commandes uniquement sur `/joint_command`.
6. `calibrated_test.py` : le banc physique de verification. Les positions reelles sont utilisees pour mesurer le resultat, jamais pour choisir les mouvements.

## Lancer

Depuis la racine du projet :

```powershell
.\bonus\lancer-classique.ps1
.\bonus\lancer-classique.ps1 -Demarrer
```

RViz montre le bras et l image de la camera. Le lancement ne demarre pas automatiquement les mouvements. Pour arreter :

```powershell
docker compose -f bonus/compose.classic.yml exec -T brain bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 service call /brain/stop std_srvs/srv/Trigger {}'
```

Pour remettre les positions initiales : ` .\bonus\lancer-classique.ps1 -Reset `.

## Mesurer

```powershell
.\bonus\tester-classique.ps1 -Graine 0 -Cubes 6
.\bonus\tester-classique.ps1 -Graine 3 -Cubes 6 -Video
```

Le resultat est vrai seulement si les six cubes sont a moins de 12 mm des cibles, sont droits, et bougent de moins de 2 mm pendant cinq secondes finales. Un depot visuellement confirme ne suffit pas a valider le test physique.

Les autres scripts sont des diagnostics conserves pour expliquer les corrections. En particulier, `fixture_test.py` prepare le socle pour isoler les derniers depots : ce n est jamais compte comme une construction complete.

## Limites

Les positions sont aleatoires parmi douze cases, avec une petite variation. Les faces des cubes sont preparees dans le plan de prise ; les orientations arbitraires ne sont pas validees. Le banc physique peut essayer trois prises par destination. La version ROS s arrete a la premiere perte de prise. Aucun essai sur un bras reel n a ete effectue.

# Reproduire le diagnostic et les essais SO-ARM101

Les essais sont exclusivement en simulation. Le conteneur proposé n'a ni réseau
externe ni périphérique série. Le pilote est instancié avec `use_sim=true` et le
banc refuse toute connexion matérielle.

## Environnement Windows / PowerShell

Depuis la racine du projet, avec Docker Desktop démarré :

```powershell
docker build -f docker/Dockerfile.control -t tek5/control .
docker build -f docker/Dockerfile.perception -t tek5/perception .
.\tools\capture_baseline.ps1 -Revision 189020ccc6f7f2eb9dd2f5adedfaeaf78e67d335
$Robotique = (Get-Location).Path
docker run -d --name so101-grasp-repro --network none `
  -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID=173 `
  -v "${Robotique}:/workspace" `
  -v "${Robotique}/ros2_ws:/ros2_ws" `
  -v "${Robotique}/sim:/opt/so101/sim" `
  tek5/control sleep infinity
docker exec so101-grasp-repro bash -lc 'source /opt/ros/jazzy/setup.bash && cd /ros2_ws && colcon build --symlink-install'
```

Si ce nom de conteneur existe déjà, choisir un autre nom dans les commandes.
L'installation du simulateur est éditable : les lancements ROS utilisent bien
le code du volume `/opt/so101/sim`, et non une ancienne copie du paquet.

Versions de calcul fixées : MuJoCo 3.10.0, ikpy 3.4.2, NumPy 1.26.4 et
OpenCV 4.10.0.84. Le Dockerfile d'origine installait ikpy 4.4.4 au moment du
diagnostic ; cette version refusait le chargement implicite de l'URDF à deux
extrémités. Les comparaisons physiques avant/après emploient le même environnement
compatible, pas deux versions différentes du moteur.

## Tests déterministes

```powershell
docker exec so101-grasp-repro bash -lc 'source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && python3 /workspace/tools/test_grasp_regressions.py'
docker exec so101-grasp-repro python3 /workspace/tools/diagnose_geometry.py
docker exec so101-grasp-repro python3 /workspace/tools/probe_collision_surfaces.py
```

Le dernier outil compare les contacts aux triangles STL visibles d'origine.
Les tests couvrent notamment FK/repères, résidus IK, couverture de collision,
conservation des masses et gains moteurs, accélération des références,
horodatages périmés, fausses confirmations de levage/dépôt, ouverture et
dégagement vertical avant retour vers Home.

## Campagnes physiques par caméra et vrais topics ROS

```powershell
docker exec so101-grasp-repro bash -lc 'source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && python3 /workspace/tools/validate_pick_place.py --baseline --seconds 25 --output /workspace/artifacts/repro_before'
docker exec so101-grasp-repro bash -lc 'source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && python3 /workspace/tools/validate_pick_place.py --seconds 180 --output /workspace/artifacts/repro_after'
```

Sans `--seeds`, les seeds **0 à 19** sont exécutés. Le tirage original est conservé :
rayon uniforme 0,18–0,30 m, angle −50 à 60 degrés et yaw 0–90 degrés. Le banc ne
change pas les poses pour faciliter une démonstration.

Les nœuds driver/perception/brain sont les vrais nœuds ROS. Les commandes passent
par `/joint_command`. Pour rendre les essais déterministes, leurs timers sont
remplacés par un ordonnanceur de test avec `/clock`, contrôle à 20 Hz, caméra à
10 Hz et intégration physique à 2 ms. Le pilote ne reçoit aucune position réelle
du cube. Cette position est réservée aux mesures de l'évaluateur.

Cette comparaison neutralise les anciennes avances de simulation déclenchées par
les lectures/commandes du backend. Elle mesure le comportement sur une même base
de temps physique ; elle ne mesure pas les performances du planificateur ROS en
temps réel.

Le prévol IK tourne dans un worker indépendant, avec sa propre chaîne. Le brain
continue de commander Home et de recevoir les observations. Le banc suspend
uniquement son horloge de plante pendant ce calcul pour que le temps CPU ne
modifie pas les conditions physiques initiales. La face de prise est choisie
après validation de l'ensemble du trajet. Le transport suit un arc autour de
l'axe d'épaule et son timeout dépend du parcours et des vitesses bornées.

Chaque seed produit `trace.json`, `result.json` et `final_camera.png` : positions
réelle/estimée du cube, FK et TCP MuJoCo, cibles/résidus IK, articulations
commandées/envoyées/atteintes, angle et ouverture de pince mesurée sur les STL,
contacts/forces, hauteurs et pics de pénétration par phase.

Une réussite exige simultanément :

- une détection par l'image ;
- un levage réel d'au moins **5 cm** et au moins **0,2 s** de contacts bilatéraux à cette hauteur ;
- un maintien au-dessus de cette hauteur pendant **tout le transport**, mesuré à chaque pas physique ;
- le cube entier contenu dans la boîte après stabilisation ;
- aucune pénétration cube/robot/sol ni collision robot/robot supérieure à **3 mm**.
- le contrôle agrégé inclut aussi les contacts robot/boîte et robot/base.

Le seuil de contact de 3 mm est un critère conservateur d'acceptation du banc,
pas une modification de la physique. Les petites pénétrations des contacts
souples MuJoCo sont enregistrées. Un essai qui finit dans la boîte mais dépasse
ce seuil reste un échec. Une chute lors d'une tentative antérieure reste incluse
dans les mesures du cycle : les récupérations ne masquent pas les échecs.
`FINISHED` ne suffit jamais. Le code de sortie est 0 seulement si le taux est
au moins 90 %, 1 sinon, et 2 si les sources ont changé pendant la campagne.

Pour figer également les entrées du banc :

```powershell
docker exec so101-grasp-repro python3 /workspace/tools/freeze_validation_source.py --output /workspace/artifacts/repro_source
docker exec so101-grasp-repro bash -lc 'source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && python3 /workspace/artifacts/repro_source/tools/validate_pick_place.py --output /workspace/artifacts/repro_frozen'
```

Le snapshot est identifié par `source_manifest.json`. Choisir un nouveau dossier
pour chaque snapshot ; l'outil refuse d'écraser un snapshot existant.

Après les commandes `repro_before` et `repro_frozen` ci-dessus, agréger sa propre
reproduction avec :

```powershell
docker exec so101-grasp-repro python3 /workspace/tools/summarize_grasp_validation.py --before /workspace/artifacts/repro_before --after /workspace/artifacts/repro_frozen --snapshot /workspace/artifacts/repro_source --output /workspace/artifacts/repro_comparison
```

Les preuves de la campagne terminée se résument avec :

```powershell
docker exec so101-grasp-repro python3 /workspace/tools/summarize_grasp_validation.py --before /workspace/artifacts/before20 --after /workspace/artifacts/verified6_a /workspace/artifacts/verified6_b --snapshot /workspace/artifacts/final_source_v6 --output /workspace/artifacts/accepted_final
```

Cette commande vérifie les 20 seeds, les hashes du snapshot, les sources courantes
et les positions initiales. Son code de sortie **1 est attendu pour la campagne
mesurée à 85 %**. Les résultats bruts restent conservés. Le détail et les limites
de ce résultat sont dans [GRASP_DIAGNOSIS.md](GRASP_DIAGNOSIS.md).

## Voir un cycle

```powershell
docker exec so101-grasp-repro bash -lc 'source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && python3 /workspace/tools/validate_pick_place.py --seeds 0 --video --output /workspace/artifacts/repro_video'
```

Ouvrir ensuite `artifacts/repro_video/seed_000/camera.mp4`. Ce film emploie la
caméra réelle de la perception. Les coordonnées réelles dans les traces sont
des diagnostics, pas des entrées applicatives.

Pour lancer les timers ROS normaux, utiliser trois terminaux du même conteneur :

```powershell
docker exec -it so101-grasp-repro bash -lc 'source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch so101_driver driver.launch.py use_sim:=true use_sim_time:=true sim_seed:=0'
docker exec -it so101-grasp-repro bash -lc 'source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch so101_perception perception.launch.py use_sim_time:=true'
docker exec -it so101-grasp-repro bash -lc 'source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch so101_brain brain.launch.py use_sim_time:=true'
```

Ne pas faire tourner simultanément le banc déterministe et ces lancements sur
le même domaine ROS : ils publieraient les mêmes topics. RViz peut être lancé
avec la configuration existante si un affichage X11 est disponible.

En simulation, **les trois nœuds doivent partager `use_sim_time=true`**. Le pilote
publie `/clock` depuis le temps physique de MuJoCo, et son timer moteur utilise
une horloge monotone indépendante. Les délais et timestamps restent ainsi liés
à la plante, même si l'horloge système de la VM est ajustée. Le banc déterministe
fournit lui-même `/clock` et désactive cette publication interne.

Pour vérifier un lancement avec les timers normaux, sans trois terminaux :

```powershell
docker exec so101-grasp-repro bash -lc 'source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && python3 /workspace/tools/smoke_ros_processes.py --seed 0 --seconds 180 --output /workspace/artifacts/ros_processes_repro.json'
```

Cet outil lance les trois processus, surveille leurs états et les arrête à la
fin. Il complète le banc physique ; son seul état `FINISHED` ne remplace pas
l'acceptation des 20 cycles. Pour revenir à une horloge système avec un backend
matériel, les trois paramètres `use_sim_time` doivent être faux ; aucun essai
matériel n'a été effectué ici.

## Géométries, force et limites de portée

Le TCP représente le centre nominal d'une prise de 50 mm, à 25 mm de la face
interne du doigt fixe et 8 mm au-dessus de l'ancien repère de bout de doigt.
L'URDF et le site MuJoCo possèdent désormais la même position et les mêmes axes.

La collision de la mâchoire mobile est décomposée depuis son **STL original**.
Le manifeste donne la source, son SHA-256, les parties et leurs hashes. Pour
la régénérer, indépendamment du runtime :

```powershell
docker tag tek5/control so101-grasp-validation
docker build -f docker/Dockerfile.geometry -t so101-grasp-geometry .
docker run --rm --network none -e OMP_NUM_THREADS=2 -v "${Robotique}:/workspace" so101-grasp-geometry python3 /workspace/tools/build_jaw_collision.py
```

L'outil utilise [CoACD](https://github.com/SarahWeiii/CoACD), version 1.0.14,
seuil métrique 0,5 mm, seed 0, sans extrusion. Il vérifie que la surface STL
d'origine n'est pas perdue à plus de 0,75 mm. Les fichiers générés sont fournis :
aucune décomposition n'est nécessaire pour lancer un essai.

Le pilote simulé accepte le champ `effort` du joint `gripper` sur
`/joint_command` : une valeur négative demande un serrage en Nm ; une valeur
nulle ou l'absence de champ reprend la commande en position. Le backend traduit
cette consigne à chaque pas moteur avec les gains existants. Les `forcerange`,
gains, masses, frottements, collisions du cube et de la boîte restent inchangés.
Aucune soudure ni égalité d'attachement n'est ajoutée.

Le backend matériel fourni n'expose pas de contrôle d'effort calibré. Il conserve
la position de repli et signale `UNSUPPORTED_CAPABILITY`. La prise matérielle
n'est donc pas validée par ces essais. Aucun robot physique n'a été connecté.

La caméra et la boîte sont calibrées pour `scene_tek5.xml`. La présence marron
de la boîte est vérifiée dans l'image avant publication de sa pose fixe.
Déplacer la caméra/boîte demande de mettre à jour les paramètres de calibration.
La perception emploie la géométrie connue de ce cube perforé de 50 mm et les
observations FK pour modéliser l'occlusion des doigts ; toute pose exige encore
une correspondance géométrique à l'image et un horodatage source frais.

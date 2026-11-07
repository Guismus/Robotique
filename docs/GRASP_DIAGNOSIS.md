# Diagnostic et correction de la saisie SO-ARM101

**Bilan final : 17/20 réussites strictes (85 %), contre 0/20 avant correction.**
L'objectif de 90 % n'est pas atteint. Les 32 régressions passent ; les trois
échecs physiques restent détaillés et comptés ci-dessous.

## Périmètre et provenance

Le cadre de `docs/SUJET.md` a été relu. Cette intervention concerne seulement
la phase 1 en simulation. Les trois nœuds sont conservés, toutes les commandes
du brain passent par `/joint_command`, et le brain/perception n'accèdent ni au
backend ni à la position MuJoCo du cube. Les accès à la vérité terrain sont
réservés aux outils de mesure sous `tools/`. Aucun robot physique n'a été connecté.

Référence avant correction : commit
`189020ccc6f7f2eb9dd2f5adedfaeaf78e67d335`, extrait avec `git show`, sans modifier
la branche. Les 20 seeds 0–19 utilisent exactement le même tirage initial
avant/après. La scène, les dimensions du cube et de la boîte, les masses,
inerties, frottements, amortissements, gains et limites de force sont conservés.
Le test compare aussi les valeurs réellement chargées par MuJoCo.

Les mesures finales viennent de `artifacts/verified6_a` et `verified6_b`, à partir
du snapshot `artifacts/final_source_v6`. Six cycles terminés avant la pause ont
été conservés ; les quatorze autres ont été exécutés à la reprise avec les mêmes
sources. L'agrégation collecte exactement les 20 `result.json`, même si les
résumés des deux reprises ne contiennent que leurs sept nouveaux cycles chacun.

Les dossiers `validated_a`, `validated_b`, `accepted`, `iteration*`, `candidate_*`,
`final_a`, `final_b` et snapshots précédents conservent les expériences antérieures.
Les preuves sont locales et ignorées par Git. La livraison sur `CubePos` conserve
le premier commit publié par l'utilisateur, `9903e81`, puis ajoute les corrections
d'exécution ROS et les documents de fonctionnement et de présentation.

## Causes démontrées

### Le résultat initial annonçait une réussite sans prise

Sur les 20 cycles initiaux : **0/20 prises/transport/dépôts valides**, alors que
le brain atteignait `FINISHED` **20/20**. Le seed 6 terminait fortuitement dans
la boîte, sans satisfaire la saisie et le maintien physiques. Un état terminal
ne constitue donc pas une preuve de succès.

Sur le seed 0, le cube réel démarre à `(0.240471, -0.089065, 0.024948)` m.
La première estimation vaut `(0.243779, -0.070787, 0.025)` m, soit environ
18,6 mm d'erreur. Pendant `GRASP`, le masque confond ensuite la boîte marron
avec le cube et publie `(0.051408, -0.216744, 0.025)` m. L'ancien `LIFT`
recalcule sa cible sur cette nouvelle observation et balaie latéralement.
La pince finit fermée à environ −0,174 rad, vide. Le cube finit hors boîte à
`(0.133395, -0.276557, 0.024959)` m ; la pénétration maximale atteint 10,72 mm.
Preuves : `artifacts/before/seed_000/trace.json`, `result.json` et captures caméra.

### Le repère nominal n'était pas le centre de prise

La FK URDF et le site MuJoCo coïncidaient déjà en translation à environ 2 µm,
mais leurs axes différaient de 90°. Cette différence d'axes n'était pas, seule,
une preuve de l'échec physique : le site n'actionne aucun moteur.

Le problème géométrique effectif était l'emplacement du repère sur le doigt
fixe, au lieu du centre d'une ouverture de 50 mm. Le TCP nominal est maintenant
à 25 mm de sa face interne et 8 mm au-dessus de l'ancien bout de doigt.
La cible ajoute 3 mm de jeu du côté fixe, dans le repère de prise choisi selon
le yaw observé du cube. Le site et l'URDF ont désormais les mêmes axes.
L'ancien ajout vertical de 18 mm au centre détecté est remplacé par cette cible
géométrique ; il n'est plus conservé comme une correction empirique de hauteur.
Preuve avant : `artifacts/geometry_before.json` ; après : test FK sur 20 poses.

`gripper=0` correspond à −0,174533 rad, fermé ; `100` à 1,74533 rad, ouvert.
Une ouverture de 55 % suffit à l'approche et évite le repli extrême de la mâchoire.
L'ouverture physique au plan de prise est mesurée par rayons sur les STL dans
chaque trace ; le pourcentage moteur n'est pas présenté comme une distance.

### Le filtre de distance ne démontrait pas la cause initiale

L'ancien filtre gardait seulement les estimations de rayon 0,10–0,38 m. Les poses
initiales testées ont un rayon de 0,18–0,30 m, et les 20 cycles avant correction
publient une détection. Ce filtre ne constitue donc pas la preuve de l'échec de
prise reproduit. Il a néanmoins été retiré du suivi : une observation visible
reste une observation, et la faisabilité du déplacement est décidée par l'IK.
Une estimation perdue ou rejetée expire maintenant, au lieu de rester publiée
comme une cible supposée fraîche.

### La collision mobile créait une surface absente du STL

MuJoCo utilisait l'enveloppe convexe d'un STL non convexe pour la mâchoire mobile.
Les premiers contacts étaient jusqu'à **4,72 mm** de la surface triangulée
visible (moyenne environ 3,99 mm), créant une cale artificielle qui poussait le
cube avant de le serrer correctement.

Une décomposition convexe reproductible du **même STL** remplace cette seule
collision : 458 parties, CoACD 1.0.14, seuil métrique 0,5 mm, seed 0, sans extrusion.
La comparaison des contacts aux triangles d'origine donne un écart maximal de
**0,279 mm**, moyen d'environ 0,056 mm. La couverture de la surface d'origine
est vérifiée ; le manifeste contient les hashes source et parties.
Preuves : `artifacts/collision_surface_probe.json`,
`artifacts/collision_surface_probe_decomposed.json` et manifeste versionné.
Ce changement restaure la géométrie de collision ; il ne réduit ni la taille du
cube ni les forces nécessaires. La collision du doigt fixe conserve une
approximation d'environ 1,6 mm, limite restante du modèle.

### Le maintien et le trajet devaient être contrôlés

Une fermeture en position continuait à comprimer un cube déjà en contact.
Le brain détecte maintenant la charge moteur, arrête la fermeture progressive,
puis demande un serrage de **0,5 Nm** pendant levage/transport/dépôt. Le pilote
simulé traduit cet effort avec les gains et limites d'origine, à chaque pas
moteur, sans lire l'objet ni les contacts. Le banc géométrique séparé montrait
qu'un serrage de 0,3 Nm pouvait lever mais perdre le cube pendant le transport ;
0,5 Nm conservait la prise avec un pic de pénétration de 2,375 mm. Ce banc de
géométrie utilise des poses de diagnostic et ne compte pas dans les 20 essais.

Un trajet cartésien direct depuis certains seeds traversait une zone centrale
non atteignable ; un timeout fixe de 12 s pouvait abandonner un transport encore
valide. Le prévol évalue désormais quatre faces équivalentes du cube et tous les
points approche/descente/levage/transport. L'arc de transport contourne l'axe
d'épaule. Les timeouts dépendent de la longueur articulaire et des vitesses
bornées. Le calcul tourne hors de la boucle de contrôle, avec une chaîne IK
indépendante. Chaque solution vérifie un résidu de position ≤2 mm et un résidu
d'orientation ≤6° à la prise, ≤28° ailleurs, sans repli IK non contraint.

Enfin, le retour direct vers Home après ouverture laissait le doigt fixe engagé
dans le cube perforé : des essais mesuraient environ 6 mm de pénétration et un
cube tiré hors boîte pendant `RETRACT`. L'ouverture est maintenant attendue
sur le feedback, puis `CLEAR` dégage verticalement les doigts avant le retour.
La hauteur de dépôt tient compte de l'inclinaison visuelle du cube, avec 8 mm
de dégagement pour limiter l'impact de lâcher.

### Le lancement ROS normal devait aussi respecter le temps de la plante

Le banc déterministe masque volontairement les variations de durée de calcul.
Un contrôle complémentaire avec les trois vrais processus et leurs timers a
donc été exécuté. Le rendu logiciel OSMesa occupait le driver jusqu'à 0,60 s,
créant un trou comparable dans la publication articulaire. Le rendu utilise
désormais une copie de `MjData` et un groupe de callbacks séparé ; seul le
copiage d'état est synchronisé avec l'intégration physique. Le timestamp de
l'image est pris avant le rendu, pas à sa fin.

Les mesures ont également montré des changements de temps ROS supérieurs au
temps monotone écoulé, dont un écart d'environ 2,5 s entre deux états. Le driver
publie maintenant `/clock` depuis le temps MuJoCo lorsqu'il utilise le temps
simulé. Un timer moteur monotone avance la physique même avant le premier
message d'horloge. Brain et perception doivent partager `use_sim_time=true`.
Le banc déterministe désactive cette publication interne puisqu'il fournit
lui-même `/clock`.

Enfin, la première publication INFO d'une transition occupait la boucle près
de 0,3 s dans les traces, tandis que la publication de commande restait sous
quelques millisecondes. Les transitions ordinaires sont au niveau DEBUG.

Preuves avant : `artifacts/ros_timing_probe.json`, `ros_transition_probe.json`.
Après : `artifacts/ros_final_image.json` atteint `FINISHED` avec les trois
processus, sans perte de feedback. Ce test emploie le simulateur embarqué dans
l'image reconstruite, sans volume sur `/opt/so101/sim`. Les trois processus
sortent 0. L'évaluateur physique indépendant reste nécessaire pour le score.

## Corrections et points d'entrée

| Fichier | Fonctions / comportement |
|---|---|
| `so101_brain/brain_node.py` | `_tick`, `_converged`, `_holding`, `_fail` : feedback, fraîcheur, confirmation visuelle, récupération bornée et échecs explicites ; `_activate_motion` : points IK prévalidés et timeouts ; `_send_joint_command` : trajectoires et effort optionnel sur le topic existant |
| `so101_brain/kinematics.py` | `Kinematics.solve` : bornes et résidus ; `plan_pick` : face et faisabilité complète ; `transport_arc` : contournement de la zone centrale |
| `so101_perception/perception_node.py` | `_cb_image`, `_pose`, `_publish_positions` : timestamps source, expiration, suivi 6D, hypothèses de support vérifiées sur l'image, confirmation stable du dépôt |
| `so101_perception/vision.py`, `occlusion.py` et JSON associés | silhouette du cube connu, pixels tronqués par le cadre, occlusion de la pince et des parois, séparation rouge/marron |
| `so101_driver/driver_node.py` | observations complètes, efforts optionnels calibrés, timer moteur monotone, `/clock` MuJoCo, caméra dans un groupe de callbacks séparé, timestamp avant rendu |
| `sim/so101_sim/__init__.py` | `step_once`, `advance`, `set_gripper_effort` : trajectoires bornées et actionneurs d'origine ; `render_camera` : copie d'état indépendante ; `simulation_time` : temps physique public ; intrinsics corrigées du demi-pixel |
| Fichiers launch des trois paquets et `so101_driver/package.xml` | paramètre `use_sim_time` explicite, seed reproductible et dépendance `rosgraph_msgs` |
| URDF et `so101_tek5.xml` | repère TCP cohérent et collision mobile décomposée ; aucune attache du cube |
| Docker / dépendances | versions compatibles fixées, simulateur installé en éditable pour que les volumes ROS utilisent bien le code courant |
| `tools/` | reproduction de l'ancien code, instrumentation physique, 32 régressions, génération géométrique, snapshot, agrégation et test des processus ROS normaux |

## Méthode d'acceptation

Les nœuds et les échanges DDS sont réels. Le banc remplace uniquement les timers
par une horloge déterministe : brain à 20 Hz, caméra à 10 Hz et MuJoCo à 2 ms.
L'horloge de plante attend le prévol IK ; ce banc ne mesure donc pas le débit
ROS en temps réel. Aucune pose réelle du cube n'est envoyée aux nœuds.

La réussite exige détection par l'image, contacts bilatéraux à hauteur pendant
au moins 0,2 s, levage réel ≥5 cm, maintien ≥5 cm pendant **tout** le transport,
cube entier dans la boîte après stabilisation et pénétration maximale **<3 mm**.
Le maintien et les contacts sont inspectés à chaque pas physique pendant le
cycle ; les poses, commandes et résidus sont enregistrés à 20 Hz. Un dernier
repos de 1 s après `FINISHED` sert à mesurer la position finale.
Le test d'inclusion emploie une enveloppe conservative de cube de 50 mm tournée
avec sa vraie orientation. Les récupérations ne remettent pas les minima/pics à
zéro. Un dépassement suffit à invalider tout le cycle.

Le banc V6 et l'agrégateur prennent en compte tous les contacts impliquant le
cube ou le robot, y compris robot/boîte et robot/base. L'agrégateur réapplique le
seuil global de 3 mm aux pics bruts de chaque phase, sans réécrire les mesures.

## Résultat mesuré : 17/20, soit 85 % — objectif 90 % non atteint

| Mesure | Avant | Après |
|---|---:|---:|
| Une détection publiée, quelle que soit sa précision | 20/20 | 20/20 |
| Levage réel ≥5 cm, même transitoire | 9/20 | 19/20 |
| Maintien ≥5 cm pendant tout le transport | 0/20 | 19/20 |
| Saisie physique et transport confirmés | 0/20 | 19/20 |
| Cube entier dans la boîte | 1/20 | 19/20 |
| Tous les contacts sous 3 mm | — | 18/20 |
| Succès physique complet avec contrôle des collisions | **0/20** | **17/20** |

Après correction, l'erreur de détection initiale moyenne est de **0,535 mm**, au
maximum **1,063 mm**. L'erreur d'approche FK par rapport à sa cible IK est en
moyenne **0,156 mm**, au maximum **0,201 mm**, sur les 19 approches abouties.
Ce dernier chiffre mesure le suivi de la cible ; il ne remplace pas la mesure
séparée de l'erreur de perception. L'écart FK/site MuJoCo maximal est de 2,184 µm.

| Seed | Détection : erreur mm | Approche : erreur mm | Contacts bilatéraux hauts : s | Levage max : cm | Transport min Δz : cm | Dans boîte | Pénétration max : mm | Succès strict |
|---:|---:|---:|---:|---:|---:|:---:|---:|:---:|
| 0 | 0,56 | 0,16 | 12,53 | 8,31 | 7,82 | oui | 2,53 | PASS |
| 1 | 0,99 | 0,16 | 24,43 | 8,32 | 7,88 | oui | 2,36 | PASS |
| 2 | 0,79 | 0,12 | 13,03 | 8,33 | 7,86 | oui | 2,32 | PASS |
| 3 | 0,89 | 0,10 | 12,95 | 8,33 | 7,88 | oui | 2,43 | PASS |
| 4 | 0,26 | 0,19 | 16,82 | 8,32 | 7,89 | oui | 3,58 | FAIL |
| 5 | 0,41 | MISSING | 0 | 0 | MISSING | non | 0,05 | FAIL |
| 6 | 0,63 | 0,14 | 14,12 | 8,37 | 7,87 | oui | 2,37 | PASS |
| 7 | 0,09 | 0,17 | 22,11 | 8,39 | 7,84 | oui | 3,17 | FAIL |
| 8 | 0,48 | 0,14 | 24,61 | 8,32 | 7,87 | oui | 2,37 | PASS |
| 9 | 0,51 | 0,19 | 13,34 | 8,35 | 7,88 | oui | 2,33 | PASS |
| 10 | 0,33 | 0,20 | 11,50 | 8,33 | 7,86 | oui | 2,42 | PASS |
| 11 | 1,06 | 0,10 | 17,72 | 8,35 | 7,89 | oui | 2,45 | PASS |
| 12 | 0,18 | 0,13 | 24,21 | 8,32 | 7,87 | oui | 2,42 | PASS |
| 13 | 0,62 | 0,20 | 20,76 | 8,39 | 7,82 | oui | 2,36 | PASS |
| 14 | 0,57 | 0,18 | 14,54 | 8,34 | 7,88 | oui | 2,32 | PASS |
| 15 | 0,11 | 0,17 | 21,40 | 8,33 | 7,86 | oui | 2,93 | PASS |
| 16 | 0,83 | 0,15 | 15,38 | 8,31 | 7,84 | oui | 2,77 | PASS |
| 17 | 0,95 | 0,19 | 11,30 | 8,35 | 7,87 | oui | 2,36 | PASS |
| 18 | 0,12 | 0,13 | 20,19 | 8,32 | 7,85 | oui | 2,38 | PASS |
| 19 | 0,34 | 0,15 | 23,50 | 8,32 | 7,87 | oui | 2,61 | PASS |

Les trois échecs restent explicitement comptés :

- **Seed 4** : saisie/maintien/dépôt réussis, mais impact cube/fond de boîte de
  **3,581 mm**, pendant `RELEASE` à **36,90 s**. `FINISHED` ne valide pas ce cycle.
- **Seed 7** : même défaut au lâcher, **3,169 mm** à **42,05 s**. Le cube est bien
  dans la boîte, mais l'essai dépasse lui aussi le seuil fixé.
- **Seed 5** : la pince masque la cible pendant l'approche ; après les deux
  récupérations autorisées, `target_lost_or_moved_before_lower` termine en
  `FAILED`. Le cube reste sur la table, sans contact pince/cube. Répéter le même
  choix de face ne lève pas cette occlusion dans la version actuelle. La capture
  finale montre aussi le cube en bord de cadre, partiellement masqué par les
  doigts : `artifacts/verified6_a/seed_005/final_camera.png`.

Les deux impacts restants dépendent de la pose inclinée et de l'incertitude
monoculaire de hauteur au lâcher. Ils demandent une meilleure confirmation du
support ou un dépôt plus progressif, pas une hausse artificielle du seuil de
collision. Ces résultats ne démontrent pas que 90 % est impossible ; ils
démontrent que **cette version atteint 85 % selon les critères annoncés**.

Preuves agrégées : `artifacts/accepted_final/summary.json` et `per_seed.md`. L'outil
vérifie les 20 seeds, leur position initiale identique à l'avant, l'intégrité des
586 fichiers du snapshot et l'égalité des hashes applicatifs courants.
SHA-256 du manifeste :
`60d2824b974ca0f7afb4a8d3e53fe83fac92fab1cdc86ec364205685da2fd38b`.
Les 20 cycles sont terminés ; aucune extrapolation d'essais partiels.

## Vérifications exécutées

- **32 régressions PASS**, code de sortie 0 : `artifacts/tests_final_32.log`.
  Les trois tests complémentaires vérifient l'indépendance du rendu lent,
  l'horloge physique publiée et le timestamp d'image conservé avant rendu.
- **6 paquets colcon construits**, code de sortie 0 : `artifacts/build_accepted.log`.
- Images finales `tek5/control` et `tek5/perception` construites. L'image de
  perception séparée charge `cv_bridge`, NumPy 1.26.4, SciPy 1.11.4 et instancie
  le vrai `PerceptionNode` avec succès.
- **20 essais initiaux + 20 corrigés V6**, intégrité des sources PASS. La reprise
  3–9 sort en 1 avec les trois échecs, la reprise 13–19 en 0. L'agrégation globale
  sort en **1**, conformément au taux strict de 85 % inférieur à l'objectif.
- **Trois processus et timers ROS normaux : smoke PASS**, sorties des trois
  processus et de l'outil à 0 : `artifacts/ros_final_image.json` et `.log`.
  Des messages rclpy `Destroyable` restent visibles à l'arrêt de cet outil.
  Ils ne sont pas utilisés pour déterminer le résultat physique des 20 cycles.
- Une reproduction vidéo indépendante du **seed 0** retrouve les mêmes mesures
  et réussit : `artifacts/verified6_video/seed_000/camera.mp4`.

Versions relevées : Python 3.12.3, ROS 2 Jazzy, MuJoCo 3.10.0, ikpy 3.4.2,
NumPy 1.26.4, SciPy 1.11.4, OpenCV 4.10.0 (distribution pip 4.10.0.84).

## Limites

- Calibration caméra et boîte fixe issue de la scène fournie ; la présence
  visuelle de la boîte est exigée, mais sa pose n'est pas réestimée librement.
  Ce raccourci débloque la simulation comme permis par `SUJET.md` ; il ne remplit
  pas l'exigence finale d'une calibration personnelle pour le robot réel.
- La perception est adaptée au cube perforé connu de 50 mm et à cette scène.
  Elle conserve une ambiguïté monoculaire quand les doigts masquent des faces ;
  les images perdues ne sont pas remplacées par une pose supposée fraîche.
- Le backend matériel ne fournit pas l'effort calibré nécessaire au serrage
  validé ici : repli en position et `UNSUPPORTED_CAPABILITY`. La compatibilité
  d'architecture est conservée ; aucune efficacité matérielle n'est revendiquée.
  Le driver ne publie la caméra qu'en simulation : un flux caméra réel et son
  `CameraInfo` doivent aussi être intégrés pour un transfert matériel.
- Le seuil de pénétration de 3 mm est un critère du banc pour les contacts
  souples existants, pas une tolérance physique de fabrication ni une absence
  mathématique d'interpénétration.

Les commandes complètes de construction, reproduction avant/après, tests et
visualisation sont dans [GRASP_VALIDATION.md](GRASP_VALIDATION.md).

# Diagnostic et correction de la saisie SO-ARM101

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

Les mesures finales viennent de `artifacts/validated_a` et `validated_b`, à partir
du snapshot `artifacts/final_source_v4`. Les autres dossiers `iteration*`,
`candidate_*`, `final_a`, `final_b` et snapshots précédents sont des expériences
intermédiaires, pas le résultat final. Les fichiers de preuve sont locaux et
ignorés par Git. Aucun commit ni push n'a été effectué.

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
Preuve avant : `artifacts/geometry_before.json` ; après : test FK sur 20 poses.

`gripper=0` correspond à −0,174533 rad, fermé ; `100` à 1,74533 rad, ouvert.
Une ouverture de 55 % suffit à l'approche et évite le repli extrême de la mâchoire.
L'ouverture physique au plan de prise est mesurée par rayons sur les STL dans
chaque trace ; le pourcentage moteur n'est pas présenté comme une distance.

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

## Corrections et points d'entrée

| Fichier | Fonctions / comportement |
|---|---|
| `so101_brain/brain_node.py` | `_tick`, `_converged`, `_holding`, `_fail` : feedback, fraîcheur, confirmation visuelle, récupération bornée et échecs explicites ; `_activate_motion` : points IK prévalidés et timeouts ; `_send_joint_command` : trajectoires et effort optionnel sur le topic existant |
| `so101_brain/kinematics.py` | `Kinematics.solve` : bornes et résidus ; `plan_pick` : face et faisabilité complète ; `transport_arc` : contournement de la zone centrale |
| `so101_perception/perception_node.py` | `_cb_image`, `_pose`, `_publish_positions` : timestamps source, expiration, suivi 6D, hypothèses de support vérifiées sur l'image, confirmation stable du dépôt |
| `so101_perception/vision.py`, `occlusion.py` et JSON associés | silhouette du cube connu, pixels tronqués par le cadre, occlusion de la pince et des parois, séparation rouge/marron |
| `so101_driver/driver_node.py` | observations complètes, efforts optionnels calibrés, simulation avancée seulement par le contrôle, seed de lancement reproductible |
| `sim/so101_sim/__init__.py` | `step_once`, `advance`, `set_gripper_effort` : références à vitesse/accélération bornées et serrage via les actionneurs d'origine ; intrinsics corrigées du demi-pixel |
| URDF et `so101_tek5.xml` | repère TCP cohérent et collision mobile décomposée ; aucune attache du cube |
| Docker / dépendances | versions compatibles fixées, simulateur installé en éditable pour que les volumes ROS utilisent bien le code courant |
| `tools/` | reproduction de l'ancien code, instrumentation physique, 29 régressions, génération géométrique, snapshot et agrégation des résultats |

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

L'agrégateur inclut aussi les collisions robot/boîte présentes dans les traces
brutes, au-delà du filtre de succès initial du banc figé. Cela rend l'acceptation
plus stricte, sans réécrire les données ni modifier les essais.

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
- Le seuil de pénétration de 3 mm est un critère du banc pour les contacts
  souples existants, pas une tolérance physique de fabrication ni une absence
  mathématique d'interpénétration.

Les commandes complètes de construction, reproduction avant/après, tests et
visualisation sont dans [GRASP_VALIDATION.md](GRASP_VALIDATION.md).

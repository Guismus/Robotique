# Bonus : pyramide de cubes creux (prototype experimental)

## Etat au 18 novembre 2026

La construction physique de la pyramide n'est PAS validee. Le premier essai ROS a echoue au premier depot et s'est arrete avec ECHEC_DEPOT_VISUEL. Ne pas annoncer un taux de reussite de pyramide a partir des essais cube-vers-bac.

## Objectif demande

Dix cubes du sujet, memes maillages, dimensions 5 cm et huit pieces de collision laissant les trous ouverts. Aucun cube plein de remplacement et aucune attache artificielle.

Pyramide de face : trois rouges en bas, deux verts au milieu, un bleu au sommet. Quatre cubes de reserve (2 rouges, 1 vert, 1 bleu). A chaque lancement, les dix cubes occupent dix cases tirees parmi douze, avec une petite variation de position et une orientation aleatoire. Le bac est retire. La zone de construction est reservee.

## Comment cela fonctionne

1. Le driver envoie les images de la camera et les angles mesures.
2. Un petit reseau PyTorch apprend a classer chaque pixel : fond, rouge, vert ou bleu. Des contours extraient ensuite les candidats cubes. Les cubes masques ou qui se touchent peuvent etre manques ou fusionnes.
3. La geometrie de camera estime les positions sur la table. Ce n'est pas une estimation de profondeur apprise.
4. Le cerveau choisit la bonne couleur et calcule les mouvements. Il autorise une approche inclinee si la verticale est inaccessible.
5. Apres un depot, une verification des couleurs attendues dans l'image autorise l'etape suivante ou arrete le cycle. Ce controle est preliminaire : il ne prouve ni la stabilite ni la hauteur exacte des cubes.

Le programme utilise uniquement /joint_command pour les moteurs. Les labels issus des corps MuJoCo sont reserves a l'entrainement. L'inference utilise uniquement RGB et calibration camera. Aucun reseau independant reconnaissant une pyramide complete n'a ete entraine : la verification actuelle compare les detections du reseau au plan geometrique.

## Verification realisee

- Les 6 positions de depot et leurs approches sont accessibles au solveur avec inclinaison. La verticale seule refuse le sommet.
- PyTorch reellement entraine sur 36 images rendues, validation sur 12 autres dispositions de la meme scene.
- IoU des pixels : rouge 0,9583 ; vert 0,9587 ; bleu 0,9639. Ce ne sont pas des taux de saisie ni de detection individuelle de tous les cubes.
- Essai complet : echec au premier depot, conserve dans essai.json et essai_final.png.
- Les poids entraines sont dans cube_net.pt.

Les dispositions aleatoires ne sont pas toutes garanties saisissables : certaines cases sont a la limite de portee et certains cubes sont caches par le bras. Le controleur ignore les cibles dont le calcul d'angles echoue et s'arrete si aucune cible de la bonne couleur n'est disponible.

## Lancer depuis la racine du depot

Docker Desktop doit etre actif. L'image tek5/control provient du projet de base.

```sh
docker build -t tek5/control -f docker/Dockerfile.control .
docker compose -f bonus/compose.yml build driver
docker compose -f bonus/compose.yml up -d
docker compose -f bonus/compose.yml logs -f
```

Demarrer un essai et conserver son resultat :

```sh
docker compose -f bonus/compose.yml exec brain bash -lc 'source /opt/ros/jazzy/setup.bash && python3 /project/bonus/test_ros.py'
```

Reinitialiser les positions aleatoires : redemarrer driver ET brain. Pour arreter tous les programmes :

```sh
docker compose -f bonus/compose.yml down
```

Reentrainer : `docker compose -f bonus/compose.yml run --rm driver python3 /project/bonus/train.py`.

## Travail restant pour une demonstration reussie

Calibrer le decalage entre le centre de la pince et le cube tenu, commander la rotation du cube lors de la pose, verifier la prise apres levage, verifier les collisions de trajectoire et les appuis reels des cubes creux. Puis effectuer des essais sur plusieurs tirages aleatoires et mesurer le taux de pyramides stables. La physique d'empilement doit etre validee avant de dire que le bonus fonctionne.

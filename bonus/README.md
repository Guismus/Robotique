# Bonus : pyramide de cubes creux

## Architecture retenue

**L IA sert uniquement a voir les cubes. La FK, la IK et les trajectoires utilisent des calculs classiques.**

Le reseau PyTorch est entraine depuis zero sur nos images de simulation. Il reconnait les pixels rouges, verts et bleus. La forme connue du cube et la calibration de la camera permettent ensuite de calculer sa position.

Dix cubes creux de 5 cm sont repartis au hasard parmi douze emplacements, avec de petites variations de position. Leurs faces sont orientees dans le plan de prise. Le bras utilise trois rouges, deux verts et un bleu pour faire trois etages ; quatre cubes restent disponibles en reserve. Le bac est retire.

Les trous, les masses et les proprietes physiques des cubes sont conserves. Aucune attache artificielle ne maintient les objets.

## Lancer sur ce PC Windows

Docker Desktop doit etre actif. Si l image n existe pas encore :

```powershell
docker build -t tek5/control -f docker/Dockerfile.control .
docker compose -f bonus/compose.yml build driver
```

Ouvrir les trois programmes ROS et RViz, puis lancer la construction :

```powershell
.\bonus\lancer-classique.ps1
.\bonus\lancer-classique.ps1 -Demarrer
```

Remettre les cubes au depart : ` .\bonus\lancer-classique.ps1 -Reset `.
Arreter les conteneurs : ` docker compose -f bonus/compose.classic.yml --profile interface down `.

## Entrainement et tests

```powershell
.\bonus\entrainer.ps1
.\bonus\tester-classique.ps1 -Graine 0 -Cubes 6
.\bonus\tester-classique.ps1 -Graine 3 -Cubes 6 -Video
```

Le premier programme entraine uniquement la vision. Le test direct utilise le meme calcul de trajectoire que ROS et verifie les positions physiques finales, leur stabilite et l orientation des cubes. Les mesures sont dans `classic/results`. La video est acceleree quatre fois.

Le banc direct autorise trois essais de prise par destination. La version ROS s arrete a la premiere perte de prise. Un controle visuel de depot ne prouve pas a lui seul la hauteur : les mesures physiques du banc sont conservees separement.

## Presenter le projet

Le [compte rendu](COMPTE_RENDU_IA.md) explique la methode et les limites. Le [guide des fichiers](classic/README.md) indique quoi montrer dans le code. Les resultats numeriques definitifs sont regroupes dans `classic/results/validation_summary.json`.

Les anciennes experiences d apprentissage des mouvements sont archivees dans `rl`. Elles ne sont pas lancees par la solution actuelle. Les anciens fichiers `brain_node.py`, `planner.py` et `compose.yml` a la racine de bonus sont le prototype initial ; pour la demonstration utiliser `compose.classic.yml` et les fichiers de `classic`.

Aucun essai sur un bras reel n a ete effectue. Les orientations initiales quelconques, l eclairage reel et toutes les positions possibles ne sont pas valides par cette demonstration.

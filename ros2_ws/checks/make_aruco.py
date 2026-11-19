"""Genere les 4 marqueurs ArUco poses sur la table de la simulation (DICT_4X4_50, id 0 a 3).

Chaque image fait 320 px : carre noir de 240 px + bord blanc de 40 px.
Dans la scene, la plaque fait 13,3 cm, donc le carre noir fait 10 cm (MARKER_SIZE de la perception).
Lancer dans le conteneur control :
    python3 /ros2_ws/checks/make_aruco.py
Pour le vrai bras : imprimer ces images a la meme taille et les poser aux memes positions.
"""
import sys
import cv2

out = sys.argv[1] if len(sys.argv) > 1 else '/opt/so101/sim/so101_sim/assets/so101'
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
for ident in range(4):
    marker = cv2.aruco.generateImageMarker(dictionary, ident, 240)
    image = cv2.copyMakeBorder(marker, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255)
    cv2.imwrite(f'{out}/aruco_{ident}.png', cv2.cvtColor(image, cv2.COLOR_GRAY2BGR))
    print(f'{out}/aruco_{ident}.png')

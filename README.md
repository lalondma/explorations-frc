# explorations-frc

## Apriltags
Dossier tags: test de détection d'apriltags sur Raspberry Pi + estimation de pose de caméra (solvepnp)

### Configuration de Raspberry Pi

* Brûler l'image WPILibPi sur carte SD selon https://docs.wpilib.org/en/stable/docs/software/vision-processing/wpilibpi/installing-the-image-to-your-microsd-card.html
* Sur le Pi (mode writable choisi via l'interface web): ajuster la date et faire $ sudo apt-get update
* Installer packages manquants pour démarrer le GUI: $ sudo apt-get install raspberrypi-ui-mods
* Utiliser raspi-config pour booter en mode desktop
* rouler optionnellement camcalib.py pour obtenir la matrice de calibration de la caméra (il faut possiblement arrêter l'app vision via l'interface web qui monopolise la caméra): nécessaire si caméra autre que PiCamera utilisée. Modifier uploaded.py

### Apriltags

* Utiliser l'interface web pour téléverser uploaded.py sur le Pi
* Référence: https://pyimagesearch.com/2020/11/02/apriltag-with-python/
* Pour estimation de pose de caméra: https://github.com/mungewell/pyPSVR/blob/master/example2_simplecv/find_april.py

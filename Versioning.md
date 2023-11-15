[//]: <> (https://stackedit.io/app#)

# Versioning GIT

# Historique des Versions

## [v1.0.0] - Nouveau Projet

### Ajouts
- Premiére version du projet.
- Intégration de la détection YOLO.
- Mise en place de Yolo V4.
- Intégration d'un fichier coco.names et de toutes les dépendances nécessaire au bon fonctionnement de yolo.

### Modifications
- Aucune modification pour cette version.

### Corrections de Bugs
- Aucune correction de bug pour cette version.

## [v1.0.1] - Amélioration / tentative de test

### Ajouts
- Ajout d'OpenPose avec les dépendances nécessaire.
- Ajout d'OpenCV avec les dépendances nécessaire.

### Modifications
- Aucune modification pour cette version.

### Corrections de Bugs
- Correction d'un bug lié au lancement de yolo avec la camera de l'ordinateur.

### Problémes rencontrés
- Version de yolo trop ancienne (v4).
- Version de python trop avancé (3.12.0 -> beta)
- Dépendances manquantes et parfois dépréciés
- 

## [v2.0.0] - Nouvelle version

### Ajouts
- Ajout de nouvelles fonctionnalités.
- Ajout d'un nouveau dossier en local pour stocker la nouvelle version et ses composantes

### Modifications
- Mise à jour vers Ultralytics version 8.0.207
- Mise à jour vers opencv-python version 4.8.1.78.
- Utilisation de np avec Numpy
- Utilisation du model yolov8n-pose.pt 

### Corrections de Bugs
- Correction d'un bug lié à la gestion de yolo avec l'import de la librairie Ultralytics.
- Correction de bug lié au model utilisé par Yolo avec l'import du model yolov8n-pose.pt

## [v2.1.0] - Traitement à partir de YOLO 

### Ajouts
- Ajout des 17 points du corps dans une liste pour les traiter.
- Ajout d'un calcul de distance entre deux points.
- Ajout d'un calcul d'angle entre 3 points.
- Ajout du calcul de la position de la personne (gauche, droite ou centre).
- Ajout du calcul de la position de la tête (gauche, droite ou centre).

### Modifications
- Mise à jour de la boucle permettant la prédiction de la pose.

### Corrections de Bugs
- Correction d'un bug lié à la gestion des keypoints (obtention des points)

## [v2.2.0] - Rétrocompatibilité

### Ajouts
- Ajout d'un script pour l'UDP
- Couplage avec Unity via un tunnel UDP.
- Ajout d'un server python pour connection avec unity
- Ajout d'un script en C# pour effectué la connection avec le serveur python
- Ajout d'un script "moveBall" dans unity

### Modifications
- Aucune mise à jour effectué

### Corrections de Bugs
- Aucune bug corrigé

## [v2.3.0] - Mise à jour du script

### Ajouts
- aucun ajout effectué

### Modifications
- Fusion du script server.py et yoloV2.py
- Mise à jour du contenue en mettant en place le tunnel UDP directement avec yolo
- Mise à jour de la gestion du server et du client

### Corrections de Bugs
- Aucune bug corrigé

## [v3.0.0] - Refactoring

### Ajouts
- Ajout de class
- Ajout de definition
- Ajout de method static

### Modifications
- Mise à jour du code en faisant de l'orienté objet.

### Corrections de Bugs
- Bug lié au refactoring corrigé
# Projet d'Interaction Danse à l'opéra

Bienvenue dans le Projet d'Interaction avec Yolo et Unity ! Ce projet vise à créer une expérience de danse interactive en fusionnant la danse avec la technologie. Vous trouverez ci-dessous des instructions détaillées pour l'installation et l'exécution du projet.

## Attention si vous voulez lancer sur unity l'armature et le feu en même temps, il vous faudra changer les ports RX et TX.
## Dans le cas contraire, il faudra disable tous les autres objets que vous n'utilisez pas. 

## Installation

### 1. Prérequis

- [Python](https://www.python.org/downloads/) (version recommandée : 3.11.6)
- [Unity Hub](https://unity3d.com/get-unity/download) (pour l'environnement Unity : v2021.3.17f1 ou supérieur)

### 2. Cloner le Répertoire

```bash

git clone https://github.com/EkinOox/YOLO-Project

```

```bash

cd GuideInstallation

```

### 3. Installer les Dépendances Python

```bash

pip install -r requirements.txt

```

### 4. Configurer Unity

- Ouvrez Unity Hub.
- Ajoutez le projet en sélectionnant le dossier cloné précédemment.

```bash

cd ../CodeFolder/code/unity

```
- Une fois dans le dossier, importer le dossier "danse" dans le Unity Hub.

# Exécution du Projet

## 1. Lancer le Script Python

```bash

cd ../danseV2

```

```bash

python .\yoloV4.py

```

-si vous souhaitez faire fonctionner l'intéraction avec le feu, veuillez activer ce script en même temps que celui de YOLO dans un autre terminal (ainsi vous aurez deux terminer distincts)  : 

```bash

python .\capteursonV2.py

```

- Pour une version plus allégé avec seulement l'estimation de pose voici une commande à executé dans votre terminal : 

```bash 

yolo pose predict model=yolov8n-pose.pt source=0 show=true

```
## 2. Lancer Unity (version yolo plus complète) 

- Ouvrez Unity Hub et sélectionnez votre projet (Danse).
- Veuillez séléctionner le dossier "scène" et choisissez la scène présente dans le dossier.
- Une fois fait, la scène apparaîtra, et vous n'aurez plus qu'à lancer les scripts python.
- Puis vous pourrez appuyer sur le bouton "Play" pour lancer le projet Unity.

# Annexe 

## Pour plus de détail :

- Dans le dossier "GuideInstallation" vous pourrez retrouver un guide d'installation plus complet en cas de problème ("installation.md") ainsi que la possibilité de consulter les documentations des différentes dépendances dans le fichier "liens_utiles.md".

- Retrouver également une description du projet dans "description.md" et le versioning du projet dans "versioning.md"

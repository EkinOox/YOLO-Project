# Guide d'Installation

## Infrastructure Python

1. **Installer Python :**
   - Télécharge et installe la dernière version de Python depuis [python.org](https://www.python.org/downloads/).

2. **Installer pip :**
   - Vérifie si `pip` est installé avec Python. Si non, installe `pip` en exécutant `python -m ensurepip --default-pip`.

3. **Installer Virtualenv (Optionnel mais recommandé) :**
   - Installe `virtualenv` pour créer un environnement Python isolé :
     ```
     pip install virtualenv
     ```

4. **Créer un Environnement Virtuel (Optionnel mais recommandé) :**
   - Crée un nouvel environnement virtuel :
     ```
     virtualenv venv
     ```

5. **Activer l'Environnement Virtuel (Optionnel mais recommandé) :**
   - Active l'environnement virtuel :
     - Sur Windows : `venv\Scripts\activate`
     - Sur Linux/macOS : `source venv/bin/activate`

6. **Installer les Dépendances Python :**
   - Installe les bibliothèques nécessaires :
     ```
     pip install yolo ultralytics opencv-python numpy
     ```

## Infrastructure Unity

1. **Installer Unity Hub :**
   - Télécharge et installe Unity Hub depuis [unity.com](https://unity3d.com/get-unity/download).

2. **Installer Unity :**
   - Lance Unity Hub et installe la version d'Unity compatible avec ton projet.

3. **Créer un Projet Unity :**
   - Crée un nouveau projet Unity et configure les paramètres selon les besoins de ton projet.

4. **Configuration UDP dans Unity :**
   - Intègre le support UDP dans Unity. Pour cela, tu peux utiliser des packages disponibles sur l'Asset Store ou rechercher des solutions open source.

## Configuration de YOLO

1. **Télécharger YOLO :**
   - Télécharge la version de YOLO que tu souhaites utiliser depuis le dépôt officiel.

2. **Installer les Dépendances YOLO :**
   - Suivre les instructions d'installation spécifiques à YOLO, généralement incluses dans la documentation.

3. **Configurer YOLO avec Ultralytics :**
   - Intègre Ultralytics avec YOLO en suivant la documentation fournie avec Ultralytics.

## Test du Projet

1. **Lancer le Projet Python :**
   - Exécute le script Python principal de ton projet.

2. **Lancer le Projet Unity :**
   - Lance ton projet Unity et assure-toi qu'il est connecté au script Python via le tunnel UDP.

3. **Tester l'Interaction :**
   - Teste l'interaction en temps réel entre Python et Unity en observant la détection d'objets et les réponses en direct.

## Remarques Importantes

- Assure-toi d'avoir les droits d'administration pour installer des logiciels.
- Il est recommandé d'utiliser des versions stables et compatibles des bibliothèques et des outils pour éviter des problèmes de compatibilité.
- Consulte régulièrement la documentation des bibliothèques et des outils pour obtenir des informations à jour.

---

N'oublie pas d'adapter ces instructions en fonction des spécificités de ton projet et de mettre à jour les versions ou les liens en fonction des changements.

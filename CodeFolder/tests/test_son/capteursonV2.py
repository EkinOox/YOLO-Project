
#---------- V2 ----------#
import sounddevice as sd
import numpy as np

# Configuration de l'audio
duration = 1.0  # Durée de la capture audio en secondes
seuil_applaudissements = 0.05  # Seuil pour la détection des applaudissements

def detecter_applaudissements(indata, frames, time, status):
    """
    Fonction de callback pour détecter les applaudissements.
    """
    # Calculer l'énergie moyenne du signal
    energy = np.sum(np.square(indata)) / len(indata)

    # Comparer l'énergie au seuil de détection
    if energy > seuil_applaudissements:
        print("Applaudissements détectés ! Énergie :", energy)
        # Ajouter ici le code pour déclencher l'événement souhaité

def main():
    """
    Fonction principale pour capturer le son et détecter les applaudissements.
    """

    # Ouvrir un flux audio en entrée (microphone)
    with sd.InputStream(callback=detecter_applaudissements):
            print("Début de l'enregistrement...")
            try:
                while True:
                    pass  # Continuez à capturer des données audio indéfiniment
            except KeyboardInterrupt:
                print("Fin de l'enregistrement.")


if __name__ == "__main__":
    main()




#---------- V1 ----------#

# import sounddevice as sd
# import numpy as np

# # Configuration de l'audio
# duration = 1.0  # Durée de la capture audio en secondes

# # Seuil pour la détection des applaudissements
# seuil_applaudissements = 80  # Vous pouvez ajuster ce seuil en fonction de la sensibilité souhaitée

# def audio_callback(indata, frames, time, status):
#     if status:
#         print(status, flush=True)
#     if np.any(indata > seuil_applaudissements):
#         print("Applaudissements détectés ! Déclenchement de l'événement...")
#         # Ajoutez ici le code pour déclencher l'événement souhaité

# # Ouvrir un flux audio en entrée (microphone)
# with sd.InputStream(callback=audio_callback):
#     sd.sleep(int(duration * 1000))

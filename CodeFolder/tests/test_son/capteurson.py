import pyaudio
import numpy as np

# Configuration de l'audio
chunk_size = 1024  # Taille du buffer audio
sample_rate = 44100  # Fréquence d'échantillonnage audio (en Hz)

# Seuil pour la détection des applaudissements (80 dB)
seuil_applaudissements = 80

# Initialisation de PyAudio
p = pyaudio.PyAudio()

# Ouvrir un flux audio en entrée (microphone)
stream = p.open(format=pyaudio.paInt16,
                channels=1,  # Mono
                rate=sample_rate,
                input=True,
                frames_per_buffer=chunk_size)

while True:
    try:
        # Lire les données audio du flux
        audio_data = np.frombuffer(stream.read(chunk_size), dtype=np.int16)

        # Calculer le niveau sonore (dB)
        rms = np.sqrt(np.mean(np.square(audio_data)))
        db = 20 * np.log10(rms / 32767)  # Correction ici
        print(f"Niveau sonore : {db:.2f} dB")
        
        # Si le niveau sonore dépasse le seuil des applaudissements, déclencher un événement
        if db > seuil_applaudissements:
            print("Applaudissements détectés ! Déclenchement de l'événement...")
            # Ajoutez ici le code pour déclencher l'événement souhaité

    except KeyboardInterrupt:
        break

# Fermer le flux audio
stream.stop_stream()
stream.close()
p.terminate()

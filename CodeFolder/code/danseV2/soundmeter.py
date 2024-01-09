import time
import audioop
import pyaudio
import math

# Initialisation de PyAudio
CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1  # Utiliser 1 canal pour l'audio mono
RATE = 44100
RECORD_SECONDS = 5

# Objet PyAudio
p = pyaudio.PyAudio()
stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)

# Variables
samples_per_section = 100
sound_track = [0] * samples_per_section

while True:
    # Lire les données depuis le périphérique
    data = stream.read(CHUNK)
    reading = audioop.max(data, 2)

    # Mettre à jour la piste audio
    sound_track = sound_track[1:] + [reading]

    # Convertir la valeur en décibels
    reference_value = 1  # Vous pouvez ajuster la valeur de référence selon vos besoins
    decibels = 20 * math.log10(reading / reference_value)

    # Afficher le niveau sonore en décibels dans la console
    print(f"Niveau sonore en décibels : {decibels:.2f} dB")
    time.sleep(0.3)
# Arrêter le flux et terminer PyAudio
stream.stop_stream()
stream.close()
p.terminate()


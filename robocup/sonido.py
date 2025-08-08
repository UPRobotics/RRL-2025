import numpy as np
import pyaudio
import webrtcvad
import noisereduce as nr
import librosa
import soundfile as sf
import whisper

RATE = 16000
CHUNK_SIZE = 480
OUTPUT_WAV = "recorded_audio.wav"
CLEANED_WAV = "cleaned_audio.wav"

vad = webrtcvad.Vad(3)
p = pyaudio.PyAudio()
stream = p.open(format=pyaudio.paInt16,
                channels=1,
                rate=RATE,
                input=True,
                frames_per_buffer=CHUNK_SIZE)
stream.start_stream()

audio_buffer = []

print("Grabando... (Ctrl+C para detener)")
try:
    while True:
        data = stream.read(CHUNK_SIZE, exception_on_overflow=False)
        if vad.is_speech(data, RATE):
            audio_buffer.append(np.frombuffer(data, dtype=np.int16))
except KeyboardInterrupt:
    print("\nGrabación detenida.")
finally:
    stream.stop_stream()
    stream.close()
    p.terminate()

if audio_buffer:
    audio_data = np.concatenate(audio_buffer, axis=0)
    sf.write(OUTPUT_WAV, audio_data, RATE)

    y, sr = librosa.load(OUTPUT_WAV, sr=RATE, mono=True)
    y_clean = nr.reduce_noise(y=y, sr=RATE)
    y_clean = librosa.util.normalize(y_clean)
    sf.write(CLEANED_WAV, y_clean, RATE)

    model = whisper.load_model("tiny.en")  # modelo tiny inglés

    result = model.transcribe(CLEANED_WAV)
    print("Transcripción:", result["text"])

    # Limpiar archivos temporales
    import os
    os.remove(OUTPUT_WAV)
    os.remove(CLEANED_WAV)
else:
    print("No se grabó audio suficiente para procesar.")
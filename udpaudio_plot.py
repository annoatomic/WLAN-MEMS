import socket
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import sounddevice as sd
import time

FILL_SECONDS = 2  # den Buffer mit 0.5s bevor du Ton startest

# -- UDP-Settings: --------------------------------
UDP_IP = ""            # alle Schnittstellen
UDP_PORT = 7000
MTU = 1472

# -- Audio-Settings: ------------------------------
CHANNELS = 2
SAMPLE_WIDTH = 2       # 16-bit PCM
SR = 16000             # Sample Rate wie im ESP-Code!
FRAME_SIZE = 4         # 2x 16bit = 4 Byte pro Stereo-Frame

BUFFER_LEN = 4096      # Plotbuffer-Länge (Frames)
AUDIO_CHUNK = MTU // FRAME_SIZE

# -- Empfangspuffer: ------------------------------
audio_buffer = np.zeros((BUFFER_LEN, CHANNELS), dtype=np.int16)
buffer_pos = 0
lock = threading.Lock()

# -- Ringpuffer für Sounddevice -------------------
from collections import deque
sound_buffer = deque(maxlen=SR*10)   # 10 Sekunden


def soft_clip(x, threshold=0.95):
    """
    Sanfter Soft-Clipper für 16-Bit-Samples.
    threshold: 0.0 bis 1.0, ab wann „weich“ begrenzt wird (z.B. 0.95 = 95%)
    """
    # Normiere auf -1..1, clippe sanft, dann skaliere zurück
    norm = x.astype(np.float32) / 32767.0
    clipped = np.where(
        np.abs(norm) > threshold,
        threshold * np.sign(norm) + (1 - threshold) * np.tanh((np.abs(norm) - threshold) / (1 - threshold)) * np.sign(norm),
        norm
    )
    return (clipped * 32767).astype(np.int16)


def udp_receiver():
    global buffer_pos
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"Höre auf UDP {UDP_PORT}...")
    while True:
        data, _ = sock.recvfrom(MTU)
        samples = np.frombuffer(data, dtype='>i2').reshape(-1, CHANNELS)
        with lock:
            take = min(BUFFER_LEN - buffer_pos, len(samples))
            audio_buffer[buffer_pos:buffer_pos+take] = samples[:take]
            buffer_pos = (buffer_pos + take) % BUFFER_LEN
        sound_buffer.extend(samples.tolist())

def audio_callback(outdata, frames, time, status):
    if status:
        print(status)
    data = np.zeros((frames, CHANNELS), dtype=np.int16)
    for i in range(frames):
        try:
            data[i] = sound_buffer.popleft()
        except IndexError:
            data[i] = 0  # Stille, wenn keine Daten
    
    # **Hier Soft-Clipping anwenden:**
    outdata[:] = soft_clip(data)

def animate(i):
    with lock:
        if buffer_pos < BUFFER_LEN // 2:
            plot_data = np.vstack((audio_buffer[buffer_pos:], audio_buffer[:buffer_pos]))
        else:
            plot_data = audio_buffer
    ax1.clear()
    ax1.plot(plot_data[:, 0], label="Links", alpha=0.8)
    ax1.plot(plot_data[:, 1], label="Rechts", alpha=0.5)
    ax1.set_ylim([-32768, 32767])
    ax1.legend(loc='upper right')
    ax1.set_title("Live Audio vom ESP32 (UDP)")

if __name__ == "__main__":
    t = threading.Thread(target=udp_receiver, daemon=True)
    t.start()

    # Warte bis etwas im Soundbuffer liegt
    print(f"Warte {FILL_SECONDS}s auf Daten...")
    time.sleep(FILL_SECONDS)

    stream = sd.OutputStream(
        samplerate=SR, channels=CHANNELS, dtype='int16',
        callback=audio_callback,
        blocksize=512,  # z.B. 512 oder 1024 Samples
        latency='low'
    )
    with stream:
        fig, ax1 = plt.subplots(figsize=(10, 4))
        ani = FuncAnimation(fig, animate, interval=30)
        plt.show()
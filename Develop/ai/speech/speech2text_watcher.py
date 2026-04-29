import os
import time
from collections import deque
from glob import glob
from datetime import datetime
from transformers import pipeline

# Folder to monitor
WATCH_DIR = r"D:\dev\Autonomous_AI_robot\Develop\hardware\firmware\speaker\voice"
LOG_FILE = os.path.join(WATCH_DIR, "speech2text.log")
QUEUE_SIZE = 3
AUDIO_EXT = (".wav", ".mp3", ".flac")

# Initialize Hugging Face pipeline (English)
asr = pipeline("automatic-speech-recognition", model="openai/whisper-base.en")

# FIFO queue for recent files
recent_files = deque(maxlen=QUEUE_SIZE)

# Helper: get sorted list of audio files by mtime
def get_audio_files():
    files = [f for f in glob(os.path.join(WATCH_DIR, "*")) if f.lower().endswith(AUDIO_EXT)]
    files.sort(key=lambda x: os.path.getmtime(x))
    return files

# Helper: write log
def write_log(audio_path, text):
    with open(LOG_FILE, "a", encoding="utf-8") as f:
        f.write(f"[{datetime.now()}] {os.path.basename(audio_path)}: {text}\n")

print("[AI] Speech-to-text watcher started.")

while True:
    audio_files = get_audio_files()
    # Add new files to queue
    for f in audio_files:
        if f not in recent_files:
            recent_files.append(f)
            # If queue overflows, remove oldest
            while len(recent_files) > QUEUE_SIZE:
                to_remove = recent_files.popleft()
                try:
                    os.remove(to_remove)
                    print(f"[AI] Removed old file: {to_remove}")
                except Exception as e:
                    print(f"[AI] Error removing {to_remove}: {e}")
            # Run ASR
            print(f"[AI] Transcribing: {f}")
            try:
                result = asr(f)
                text = result["text"] if isinstance(result, dict) else result
                write_log(f, text)
                print(f"[AI] Result: {text}")
            except Exception as e:
                print(f"[AI] ASR error for {f}: {e}")
    # Remove files not in queue
    for f in audio_files:
        if f not in recent_files:
            try:
                os.remove(f)
                print(f"[AI] Removed extra file: {f}")
            except Exception as e:
                print(f"[AI] Error removing {f}: {e}")
    time.sleep(2)

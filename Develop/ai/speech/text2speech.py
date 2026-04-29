import os
from datetime import datetime
from gtts import gTTS

def text_to_speech(text, out_path, lang="en"):
    tts = gTTS(text=text, lang=lang)
    tts.save(out_path)
    print(f"[TTS] Saved: {out_path}")

if __name__ == "__main__":
    # Example usage
    text = "Hello, this is a text to speech test."
    out_dir = r"D:\dev\Autonomous_AI_robot\Develop\hardware\firmware\speaker\voice"
    os.makedirs(out_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_file = os.path.join(out_dir, f"tts_{timestamp}.mp3")
    text_to_speech(text, out_file)

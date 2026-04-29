import torch
import soundfile as sf
from transformers import WhisperProcessor, WhisperForConditionalGeneration

AUDIO_FILE = r"D:\dev\Autonomous_AI_robot\Develop\hardware\firmware\speaker\voice\Recording (4).wav"
processor = WhisperProcessor.from_pretrained("openai/whisper-tiny.en")
model = WhisperForConditionalGeneration.from_pretrained("openai/whisper-tiny.en")

# Load your audio file
audio, sr = sf.read(AUDIO_FILE)
input_features = processor(audio, sampling_rate=sr, return_tensors="pt").input_features
predicted_ids = model.generate(input_features)
transcription = processor.batch_decode(predicted_ids, skip_special_tokens=True)[0]
print("Transcription:", transcription)
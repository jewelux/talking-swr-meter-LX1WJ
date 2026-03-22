#!/usr/bin/env python3
import re
import wave
from pathlib import Path

SAMPLE_RATE = 16000
CHANNELS = 1
SAMPLE_WIDTH = 2  # 16-bit PCM


def parse_voice_arrays(header_text: str):
    pattern = re.compile(
        r"const\s+uint8_t\s+(voice_[A-Za-z0-9_]+)\[\]\s+PROGMEM\s*=\s*\{(.*?)\};",
        re.S,
    )
    voices = []
    for name, body in pattern.findall(header_text):
        hex_bytes = re.findall(r"0x([0-9A-Fa-f]{2})", body)
        voices.append((name, bytes(int(h, 16) for h in hex_bytes)))
    return voices


def write_wav(out_path: Path, pcm_bytes: bytes):
    with wave.open(str(out_path), "wb") as wav:
        wav.setnchannels(CHANNELS)
        wav.setsampwidth(SAMPLE_WIDTH)
        wav.setframerate(SAMPLE_RATE)
        wav.writeframes(pcm_bytes)


def main():
    here = Path(__file__).resolve().parent
    header_path = here / "voice_data.h"
    if not header_path.exists():
        raise FileNotFoundError(f"Missing {header_path}")

    header_text = header_path.read_text(encoding="utf-8", errors="replace")
    voices = parse_voice_arrays(header_text)
    if not voices:
        raise RuntimeError("No voice arrays found in voice_data.h")

    for name, pcm_bytes in voices:
        write_wav(here / f"{name}.wav", pcm_bytes)

    print(f"Recovered {len(voices)} wav files to {here}")


if __name__ == "__main__":
    main()

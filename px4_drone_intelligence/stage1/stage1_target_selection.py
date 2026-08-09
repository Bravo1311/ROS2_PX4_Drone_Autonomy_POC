#!/usr/bin/env python3
import argparse
import base64
import json
import os
import re
import sys

from PIL import Image
import io

import requests


SYSTEM_PROMPT = """
You are a robotics map reasoning assistant.

You are given:
1. A top-down annotated map image of an environment.
2. A natural language instruction for a drone.

Your task:
- Return ONLY valid JSON.
- Do explain your answer.
- Do not include markdown fences.
- Analyse very carefully before answering
- Distance is the path distance avoiding the walls. The path cannot go through a wall.

Output schema:
{
  "target_marker": Give here the marker ID,
      Or
  "answer": Give your answer,
  "reason_short": "Detailed reason"
}
""".strip()

# - Identify which marker the drone should go to.


def encode_image_base64(image_path: str) -> str:
    # Resize to max 512px on longest side before encoding
    img = Image.open(image_path)
    img.thumbnail((512, 512), Image.LANCZOS)
    buffer = io.BytesIO()
    img.save(buffer, format='PNG')
    return base64.b64encode(buffer.getvalue()).decode("utf-8")


def extract_json(text: str):
    text = text.strip()

    try:
        return json.loads(text)
    except json.JSONDecodeError:
        pass

    match = re.search(r"\{.*\}", text, re.DOTALL)
    if match:
        return json.loads(match.group(0))

    raise ValueError(f"Could not parse JSON from model output:\n{text}")


def validate_output(data):
    allowed = {"M0", "M1", "M2", "M3"}

    if "target_marker" not in data:
        raise ValueError("Missing target_marker")

    if data["target_marker"] not in allowed:
        raise ValueError(f"Invalid target_marker: {data['target_marker']}")

    if "reason_short" not in data:
        data["reason_short"] = ""

    return data


# def query_ollama(image_path: str, instruction: str, model: str = "minicpm-v"):
def query_ollama(image_path: str, instruction: str, model: str = "qwen2.5vl:3b"):
    image_b64 = encode_image_base64(image_path)

    prompt = f"""{SYSTEM_PROMPT}

Instruction: {instruction}

Return only JSON."""

    payload = {
        "model": model,
        "messages": [
            {
                "role": "user",
                "content": prompt,
                "images": [image_b64]
            }
        ],
        "stream": False,
        "options": {
            "num_ctx": 8192,
            "temperature": 0.1   # low temperature = more deterministic JSON output
        }
    }

    r = requests.post("http://localhost:11434/api/chat", json=payload, timeout=300)
    r.raise_for_status()

    response_text = r.json()["message"]["content"]
    print(f"DEBUG RAW:\n{response_text}", file=sys.stderr)
    data = extract_json(response_text)
    return validate_output(data)

def query_ollama_text_only(instruction: str, model: str = "qwen2.5vl:3b"):
    payload = {
        "model": model,
        "messages": [
            {
                "role": "user",
                "content": f'Reply with only this exact JSON, no other text: {{"target_marker": "M1", "reason_short": "test"}}'
            }
        ],
        "stream": False,
        "options": {"temperature": 0.1}
    }
    r = requests.post("http://localhost:11434/api/chat", json=payload, timeout=300)
    r.raise_for_status()
    response_text = r.json()["message"]["content"]
    print(f"TEXT ONLY RESPONSE:\n{response_text}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True)
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--model", default="llava")
    args = parser.parse_args()

    try:
        result = query_ollama_text_only(args.instruction, args.model)
        # result = query_ollama(args.image, args.instruction, args.model)
        print(json.dumps(result, indent=2))
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
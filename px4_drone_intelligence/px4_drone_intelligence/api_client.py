"""Provider agnostic chat client. No ROS imports"""

import os
import json
import base64
import mimetypes
import urllib.request
import urllib.error

API_MAP = {
    "openRouter":"OPENROUTER_API_KEY",
}

ENDPOINTS = {
    "openRouter": "https://openrouter.ai/api/v1/chat/completions",
    "openAI": "https://api.openai.com/v1/chat/completions",
}

DEFAULT_MODELS = {
    "openRouter": "openai/gpt-4o-mini",
    "openAI": "gpt-4o-mini",
}

SUPPORTED_IMAGE_TYPES = {"image/jpeg", "image/png", "image/gif", "image/webp"}
MAX_IMAGE_BYTES = 20 * 1024 * 1024

def resolve_key(provider, env = None):
    env = os.environ if env is None else env
    if provider not in API_MAP:
        raise ValueError(f"Unknown provider '{provider}'. Expected: {sorted(API_MAP)}")
    key = env.get(API_MAP[provider])
    if not key:
        raise ValueError(f"{API_MAP[provider]} is not set in the environment.")
    return key

def encode_image(path):
    """Read an image file and return an EFC 2397 data URL"""
    path = os.path.expanduser(path)

    if not os.path.isfile(path):
        raise ValueError(f"Image not found: {path}")

    size = os.path.getsize(path)
    if size == 0:
        raise ValueError(f"Image is Empty: {path}")
    if size > MAX_IMAGE_BYTES:
        raise ValueError(f"Image too large ({size / 1e6:.1f} MB, limit 20 MB): {path}")

    mime, _ = mimetypes.guess_type(path)
    if mime not in SUPPORTED_IMAGE_TYPES:
        raise ValueError(
            f"Unsupported image type '{mime}' for {path}."
            f"Expected one of: {sorted(SUPPORTED_IMAGE_TYPES)}")

    with open(path, "rb") as f:
        b64 = base64.b64encode(f.read()).decode("ascii")

    data_url = f"data:{mime};base64,{b64}"
    print("DEBUG encode_image ->", repr(data_url[:40]))
    return data_url, size

def _post(url, payload, key, timeout):
    req = urllib.request.Request(
        url,
        data = json.dumps(payload).encode(),
        headers={
            "Authorization": f"Bearer {key}",
            "Content-Type": "applicaiton/json",
        },
        method="POST"
    )
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return json.loads(resp.read())
    except urllib.error.HTTPError as e:
        body = e.read().decode()[:400]
        raise RuntimeError(f"HTTP {e.code}: {body}") from None
    except urllib.error.URLError as e:
        raise RuntimeError(f"Network error: {e.reason}") from None

def ask(provider, prompt, image_path=None, key=None, model=None, max_tokens=512, timeout=60):
    """Send a prompt (optionally with one image). Returns (text, meta dict)"""
    key = key or resolve_key(provider)
    model = model or DEFAULT_MODELS[provider]

    if image_path:
        data_url, _ = encode_image(image_path)
        print("DEBUG ask ->", repr(data_url[:40]))
        content = [
            {"type":"text", "text": prompt},
            {"type":"image_url", "image_url": {"url": data_url}},
        ]
    else:
        content = prompt

    payload = {
        "model": model,
        "messages": [{"role": "user", "content": content}],
        "max_tokens": max_tokens
    }

    data = _post(ENDPOINTS[provider], payload, key, timeout)

    # OpenRouter returns 200 with an error body on upstream failures
    if "error" in data:
        err = data["error"]
        code = err.get("code")
        msg = err.get("message", "unknown error")
        meta = err.get("metadata") or {}
        provider_name = meta.get("provider_name", "?")
        source = meta.get("limit_source", "")
        raise RuntimeError(
            f"API error {code} from {provider_name}"
            f"{f' [{source}]' if source else ''}: {msg}")

    if "choices" not in data or not data["choices"]:
        raise RuntimeError(f"Malformed response: {json.dumps(data)[:400]}")

    choice = data["choices"][0]
    msg = choice["message"]

    if msg.get("refusal"):
        raise RuntimeError(f"Model refused: {msg['refusal']}")

    text = msg.get("content")
    if text is None:
        raise RuntimeError(f"Empty Content (finish reason: {choice.get('finish_reason')})")

    usage = data.get("usage", {})
    meta = {
        "finish_reason": choice.get("finish_reason"),
        "model": data.get("model"),
        "total_tokens": usage.get("total_tokens"),
        "cost": usage.get("cost")
    }
    return text, meta
"""
ai_client.py — the model round-trip.

Deliberately mirrors main.py's _openai_compatible_tool_chat /
_call_model_with_tools pattern from the existing Lumexa CAD repo: a plain
urllib POST (no extra HTTP client dependency), the same message/tool_call
shape, and the same rate-limit/backoff handling — that pattern is already
battle-tested there, so there's no reason to reinvent it here.

Provider: defaults to OpenRouter running Nemotron 3 Ultra, per this
service's brief. AI_PROVIDER is kept as an env var (rather than hardcoded)
in case you later want to A/B against another OpenAI-compatible
tool-calling provider (groq, etc.) the way the CAD repo does — same
env var name intentionally, so if this ever merges into that repo's
process, the config already lines up.
"""
from __future__ import annotations

import json
import os
import time
import uuid
import urllib.request
import urllib.error

AI_PROVIDER = os.environ.get("AI_PROVIDER", "openrouter")

OPENROUTER_API_KEY = os.environ.get("OPENROUTER_API_KEY", "")
OPENROUTER_API_URL = "https://openrouter.ai/api/v1/chat/completions"
# nvidia/nemotron-3-ultra-550b-a55b — 55B active / 550B total MoE, tool-calling
# capable, per this service's brief. Verify current id/pricing/context window
# on OpenRouter's model page before deploying — these details change.
OPENROUTER_MODEL = os.environ.get("OPENROUTER_MODEL", "nvidia/nemotron-3-ultra-550b-a55b")

GROQ_API_KEY = os.environ.get("GROQ_API_KEY", "")
GROQ_API_URL = "https://api.groq.com/openai/v1/chat/completions"
GROQ_MODEL = os.environ.get("GROQ_MODEL", "openai/gpt-oss-120b")

AGENT_TURN_MAX_TOKENS = int(os.environ.get("AGENT_TURN_MAX_TOKENS", "2000"))


def _provider_endpoint():
    if AI_PROVIDER == "openrouter":
        return OPENROUTER_API_URL, OPENROUTER_API_KEY, OPENROUTER_MODEL
    if AI_PROVIDER == "groq":
        return GROQ_API_URL, GROQ_API_KEY, GROQ_MODEL
    return None, None, None


def to_openai_tools(tool_specs: list) -> list:
    return [{"type": "function", "function": {"name": t["name"], "description": t["description"],
             "parameters": t["parameters"]}} for t in tool_specs]


def _openai_compatible_tool_chat(api_url, api_key, model, messages, tools,
                                  temperature=0.2, max_tokens=AGENT_TURN_MAX_TOKENS) -> dict:
    payload = json.dumps({"model": model, "messages": messages, "tools": tools,
                           "tool_choice": "auto", "temperature": temperature,
                           "max_tokens": max_tokens}).encode()
    headers = {"Content-Type": "application/json", "Authorization": f"Bearer {api_key}",
               "User-Agent": "Mozilla/5.0 (compatible; LumexaElectronics/1.0)",
               "Accept": "application/json"}
    req = urllib.request.Request(api_url, data=payload, headers=headers)
    try:
        with urllib.request.urlopen(req, timeout=60) as resp:
            data = json.loads(resp.read())
    except urllib.error.HTTPError as e:
        body = e.read().decode(errors="ignore")
        raise RuntimeError(f"__HTTP_{e.code}__:{body}")
    except urllib.error.URLError as e:
        raise RuntimeError(f"__CONN__:{e}")

    try:
        return data["choices"][0]["message"]
    except (KeyError, IndexError, TypeError):
        raise RuntimeError(f"__SHAPE__:{json.dumps(data)[:500]}")


class ProviderError(Exception):
    def __init__(self, status_code: int, detail: str):
        super().__init__(detail)
        self.status_code = status_code
        self.detail = detail


def call_model_with_tools(messages: list, tool_specs: list,
                           temperature: float = 0.2, max_tokens: int = AGENT_TURN_MAX_TOKENS) -> dict:
    url, key, model = _provider_endpoint()
    if url is None:
        raise ProviderError(501, f"Unsupported AI_PROVIDER '{AI_PROVIDER}'. Use 'openrouter' or 'groq'.")
    if not key:
        raise ProviderError(500, f"{AI_PROVIDER.upper()}_API_KEY is not configured on the server.")

    try:
        msg = _openai_compatible_tool_chat(url, key, model, messages, to_openai_tools(tool_specs),
                                            temperature=temperature, max_tokens=max_tokens)
    except RuntimeError as e:
        text = str(e)
        if text.startswith("__HTTP_429__"):
            raise ProviderError(429, text.split(":", 1)[1])
        if text.startswith("__HTTP_"):
            code = text.split("__")[2]
            raise ProviderError(502, f"Provider error ({code}): {text.split(':', 1)[1]}")
        if text.startswith("__CONN__"):
            raise ProviderError(502, f"Provider connection error: {text.split(':', 1)[1]}")
        if text.startswith("__SHAPE__"):
            raise ProviderError(502, f"Unexpected response shape: {text.split(':', 1)[1]}")
        raise ProviderError(502, text)

    tool_calls = []
    for tc in (msg.get("tool_calls") or []):
        try:
            args = json.loads(tc.get("function", {}).get("arguments") or "{}")
        except json.JSONDecodeError:
            args = {"_raw_arguments_unparseable": tc.get("function", {}).get("arguments")}
        tool_calls.append({"id": tc.get("id") or f"call_{uuid.uuid4().hex[:12]}",
                            "name": tc.get("function", {}).get("name"), "arguments": args})
    return {"role": "assistant", "content": msg.get("content") or msg.get("reasoning"),
            "tool_calls": tool_calls, "_raw_message": msg}


def format_tool_result_message(tool_call_id: str, tool_name: str, result: dict) -> dict:
    text = json.dumps(result, default=str)
    if len(text) > 6000:
        text = json.dumps({"truncated": True, "status": result.get("status"),
                            "keys_available": list(result.keys()), "partial": text[:4000]})
    return {"role": "tool", "tool_call_id": tool_call_id, "name": tool_name, "content": text}


def parse_retry_after_seconds(detail: str, default: float = 5.0) -> float:
    """Best-effort extraction of a retry-after hint from a 429 body; falls
    back to a short fixed wait rather than failing the whole run outright."""
    import re
    match = re.search(r'"?retry[-_ ]after"?\D{0,5}(\d+(?:\.\d+)?)', detail, re.IGNORECASE)
    if match:
        try:
            return min(float(match.group(1)), 60.0)
        except ValueError:
            pass
    return default

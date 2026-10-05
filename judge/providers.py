"""The judge's providers: Anthropic and OpenAI over httpx (already in the runner's env; no SDK is installed), and a
stub for fixtures and dry runs.

One call each: the rubric as the system text, the phase + context as the user text, the frame as a base64 image; the
model's text comes back as is. If JUDGE_MODEL is unset the provider's models are listed once and one is chosen by the
rule written beside each chooser; the rule's text goes into the log with the choice.

Secrets: the key travels in a request header and nowhere else. A ProviderError's message is built from fixed words,
the HTTP status and the provider's error type (letters and underscores only) - never from a response body, a URL, a
header or an httpx exception's text (OpenAI's 401 body quotes part of the key). It is raised outside the `except`
block, so no httpx exception (which holds the request and its headers) rides along as its context.
"""
from __future__ import annotations

import base64
import re
import time
from typing import Callable, Protocol

import httpx

ANTHROPIC_URL = "https://api.anthropic.com/v1"
ANTHROPIC_VERSION = "2023-06-01"
OPENAI_URL = "https://api.openai.com/v1"
MAX_OUTPUT_TOKENS = 2048          # the reply is one small JSON object; room for a model that thinks first
LIST_PAGES_MAX = 10

ANTHROPIC_RULE = ("GET /v1/models; keep the models whose capabilities.image_input.supported is true; the list has no "
                  "speed field, so speed is read from the tier in the id: 'haiku' before 'sonnet' before 'opus' before "
                  "anything else; within the first tier present, the newest created_at")
OPENAI_RULE = ("GET /v1/models; the list has no capability or speed field, so both are read from the id: keep ids that "
               "start with 'gpt-' (not 'gpt-3') and name no other modality (audio, realtime, tts, transcribe, search, "
               "embedding, image, instruct, codex, moderation); 'nano' before 'mini' before the rest; within the first "
               "tier present, the newest created. If the chosen model takes no image the call fails and the verdict is "
               "UNSAFE")
_OPENAI_EXCLUDE = ("audio", "realtime", "tts", "transcribe", "search", "embedding", "image", "instruct", "codex",
                   "moderation")


class ProviderError(RuntimeError):
    """A provider call failed. The message never holds a key, a header, a URL or a response body."""


class Provider(Protocol):
    name: str
    model: str

    def complete(self, system: str, user_text: str, image: bytes, media_type: str, timeout_s: float) -> str: ...


def _error_type(data) -> str:
    err = data.get("error") if isinstance(data, dict) else None
    word = (err.get("type") or err.get("code") or "") if isinstance(err, dict) else ""
    return re.sub(r"[^a-z_]", "", str(word).lower())[:48]


def _request(provider: str, method: str, url: str, headers: dict, body: dict | None, timeout_s: float,
             transport: httpx.BaseTransport | None) -> dict:
    """One HTTP exchange -> the decoded JSON object, or ProviderError (see the module docstring for what it may say)."""
    problem = status = data = None
    try:
        with httpx.Client(timeout=max(0.1, float(timeout_s)), transport=transport) as client:
            r = client.request(method, url, headers=headers, json=body)
        status = r.status_code
        try:
            data = r.json()
        except ValueError:
            data = None
    except httpx.TimeoutException:
        problem = "http timeout"
    except httpx.HTTPError as e:
        problem = type(e).__name__
    except Exception as e:  # noqa: BLE001 - anything else: its class name only
        problem = type(e).__name__
    if problem is None and status != 200:
        problem = f"http {status} {_error_type(data)}".strip()
    if problem is None and not isinstance(data, dict):
        problem = "reply is not a JSON object"
    if problem is not None:
        raise ProviderError(f"{provider}: {problem}")
    return data


# ------------------------------------------------------------------------------------------ Anthropic
def _anthropic_headers(key: str) -> dict:
    return {"x-api-key": key, "anthropic-version": ANTHROPIC_VERSION, "content-type": "application/json"}


def anthropic_models(key: str, timeout_s: float, transport: httpx.BaseTransport | None = None) -> list:
    out, after = [], None
    for _ in range(LIST_PAGES_MAX):
        url = f"{ANTHROPIC_URL}/models?limit=1000" + (f"&after_id={after}" if after else "")
        page = _request("anthropic", "GET", url, _anthropic_headers(key), None, timeout_s, transport)
        out += [m for m in page.get("data") or [] if isinstance(m, dict)]
        after = page.get("last_id")
        if not page.get("has_more") or not after:
            break
    return out


def _supported(model: dict, *path: str) -> bool:
    node = model.get("capabilities")
    for k in path:
        node = node.get(k) if isinstance(node, dict) else None
    return isinstance(node, dict) and node.get("supported") is True


def choose_anthropic(models: list) -> dict:
    """ANTHROPIC_RULE. -> the chosen model's object."""
    def tier(mid: str) -> int:
        return next((i for i, t in enumerate(("haiku", "sonnet", "opus")) if t in mid), 3)
    vision = [m for m in models if isinstance(m.get("id"), str) and _supported(m, "image_input")]
    if not vision:
        raise ProviderError("anthropic: no vision-capable model in the list")
    best = min(tier(m["id"]) for m in vision)
    return max((m for m in vision if tier(m["id"]) == best), key=lambda m: str(m.get("created_at") or ""))


class AnthropicProvider:
    name = "anthropic"

    def __init__(self, key: str, model: str, effort_low: bool = False, transport: httpx.BaseTransport | None = None):
        self._key, self.model, self.effort_low, self._transport = key, model, bool(effort_low), transport

    def __repr__(self) -> str:
        return f"AnthropicProvider(model={self.model!r})"

    @classmethod
    def with_capabilities(cls, key: str, model: str, timeout_s: float,
                          transport: httpx.BaseTransport | None = None) -> "AnthropicProvider":
        """A named model: its capabilities are read once (GET /v1/models/{id}); unreadable -> no effort setting."""
        try:
            info = _request("anthropic", "GET", f"{ANTHROPIC_URL}/models/{model}", _anthropic_headers(key), None,
                            timeout_s, transport)
        except ProviderError:
            info = {}
        return cls(key, model, _supported(info, "effort", "low"), transport)

    def complete(self, system: str, user_text: str, image: bytes, media_type: str, timeout_s: float) -> str:
        body = {"model": self.model, "max_tokens": MAX_OUTPUT_TOKENS, "system": system,
                "messages": [{"role": "user", "content": [
                    {"type": "image", "source": {"type": "base64", "media_type": media_type,
                                                 "data": base64.standard_b64encode(image).decode("ascii")}},
                    {"type": "text", "text": user_text}]}]}
        if self.effort_low:                      # a model that thinks: the least of it, the budget is 8 s
            body["output_config"] = {"effort": "low"}
        data = _request("anthropic", "POST", f"{ANTHROPIC_URL}/messages", _anthropic_headers(self._key), body,
                        timeout_s, self._transport)
        blocks = data.get("content") if isinstance(data.get("content"), list) else []
        # a refusal or a cut-off reply carries no complete JSON: the judge reads it as malformed (UNSAFE)
        return "".join(str(b.get("text") or "") for b in blocks if isinstance(b, dict) and b.get("type") == "text")


# ------------------------------------------------------------------------------------------ OpenAI
def _openai_headers(key: str) -> dict:
    return {"authorization": f"Bearer {key}", "content-type": "application/json"}


def openai_models(key: str, timeout_s: float, transport: httpx.BaseTransport | None = None) -> list:
    page = _request("openai", "GET", f"{OPENAI_URL}/models", _openai_headers(key), None, timeout_s, transport)
    return [m for m in page.get("data") or [] if isinstance(m, dict)]


def choose_openai(models: list) -> dict:
    """OPENAI_RULE. -> the chosen model's object."""
    def tier(mid: str) -> int:
        return 0 if "nano" in mid else 1 if "mini" in mid else 2
    chat = [m for m in models if isinstance(m.get("id"), str) and m["id"].startswith("gpt-")
            and not m["id"].startswith("gpt-3") and not any(x in m["id"] for x in _OPENAI_EXCLUDE)]
    if not chat:
        raise ProviderError("openai: no chat model in the list")
    best = min(tier(m["id"]) for m in chat)
    return max((m for m in chat if tier(m["id"]) == best), key=lambda m: int(m.get("created") or 0))


class OpenAIProvider:
    name = "openai"

    def __init__(self, key: str, model: str, transport: httpx.BaseTransport | None = None):
        self._key, self.model, self._transport = key, model, transport

    def __repr__(self) -> str:
        return f"OpenAIProvider(model={self.model!r})"

    def complete(self, system: str, user_text: str, image: bytes, media_type: str, timeout_s: float) -> str:
        url = f"data:{media_type};base64," + base64.standard_b64encode(image).decode("ascii")
        body = {"model": self.model, "max_completion_tokens": MAX_OUTPUT_TOKENS,
                "messages": [{"role": "system", "content": system},
                             {"role": "user", "content": [{"type": "text", "text": user_text},
                                                          {"type": "image_url", "image_url": {"url": url}}]}]}
        data = _request("openai", "POST", f"{OPENAI_URL}/chat/completions", _openai_headers(self._key), body,
                        timeout_s, self._transport)
        choices = data.get("choices") if isinstance(data.get("choices"), list) else []
        msg = choices[0].get("message") if choices and isinstance(choices[0], dict) else None
        text = msg.get("content") if isinstance(msg, dict) else None
        return text if isinstance(text, str) else ""


# ------------------------------------------------------------------------------------------ stub
STUB_REASON = "stub: a scripted reply, no model looked at the frame"
_STUB_DEFAULT = {
    "CP1": '{"hand_present": false, "hand_open_waiting": false, "blade_toward_hand": null, "p_unsafe": 0.05, '
           '"recommend": "proceed", "reason": "%s"}',
    "CP2": '{"hand_present": false, "hand_open_waiting": false, "blade_toward_hand": null, "p_unsafe": 0.05, '
           '"recommend": "proceed", "reason": "%s"}',
    "CP3": '{"hand_present": true, "hand_open_waiting": false, "blade_toward_hand": false, "p_unsafe": 0.1, '
           '"recommend": "proceed", "reason": "%s"}',
    "FREEZE": '{"hand_present": true, "hand_open_waiting": true, "blade_toward_hand": false, "p_unsafe": 0.2, '
              '"recommend": "handover", "reason": "%s"}',
}


class StubProvider:
    """No network, no model. `script` is a list of replies used in order (the last one repeats) or a callable
    (phase, user_text, image) -> reply. A reply is the text to return, or {"text": ..., "delay_s": ...} to answer
    late, or an Exception to raise. With no script each phase gets a fixed well-formed reply that says it is a stub."""

    name = "stub"
    model = "stub"

    def __init__(self, script: list | Callable | None = None):
        self._script = script
        self.calls = 0

    def complete(self, system: str, user_text: str, image: bytes, media_type: str, timeout_s: float) -> str:
        self.calls += 1
        phase = user_text.split("\n", 1)[0].removeprefix("phase: ").strip()
        if callable(self._script):
            reply = self._script(phase, user_text, image)
        elif self._script:
            reply = self._script[min(self.calls, len(self._script)) - 1]
        else:
            reply = _STUB_DEFAULT.get(phase, _STUB_DEFAULT["FREEZE"]) % STUB_REASON
        if isinstance(reply, Exception):
            raise reply
        if isinstance(reply, dict):
            time.sleep(float(reply.get("delay_s") or 0.0))
            reply = reply.get("text", "")
        return str(reply)

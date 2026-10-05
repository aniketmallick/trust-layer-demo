#!/usr/bin/env python
"""Judge fixtures - every rule of the judge seen to fire and seen not to fire, with no network and no .env.

    python -m judge.fixtures            # the table; exit 0 = all as expected
    judge/tests/test_judge.py           # the same registry through pytest

Style of anchor/negative_fixtures*.py: a name, one sentence of what is injected and what must happen, a function
returning (as_expected, detail); kind="control" marks the must-not-fire cases. Providers are the stub or the real
provider classes behind an httpx.MockTransport - nothing leaves the machine. The fake key is assembled at run time
(no token-shaped literal in this file).
"""
from __future__ import annotations

import ast
import base64
import contextlib
import io
import json
import logging
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import httpx

from . import judge as jd
from . import providers as pv
from .env import Env, load_env, parse
from .scrub import redact

HERE = Path(__file__).resolve().parent
REGISTRY: list = []
PNG_1PX = base64.b64decode("iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mNkYPhfDwAChwGA60e6kgAAAABJRU5ErkJggg==")
GOOD = {"hand_present": True, "hand_open_waiting": True, "blade_toward_hand": False, "p_unsafe": 0.2,
        "recommend": "handover", "reason": "An open hand rests palm up in the place zone."}


def fixture(name: str, what: str, kind: str = "negative"):
    def deco(fn):
        REGISTRY.append({"name": name, "what": what, "kind": kind, "fn": fn})
        return fn
    return deco


def _fake_key() -> str:
    return "sk-" + "ant-" + "F4K3" * 10


@contextlib.contextmanager
def _frame(data: bytes = PNG_1PX, name: str = "frame.png"):
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / name
        p.write_bytes(data)
        yield p


def _stub(*script) -> jd.Judge:
    return jd.load(provider="stub", stub_script=list(script))


def _is_abort(v: dict, reason: str) -> bool:
    return (v["unsafe"] is True and v["p_unsafe"] == 1.0 and v["recommend"] == "abort" and v["reason"] == reason
            and set(jd.FIELDS) <= set(v))


# ------------------------------------------------------------------------------------------ the reply
@fixture("JUDGE_malformed_text_is_abort_unsafe",
         "the stub answers with text that is not JSON: the verdict is p_unsafe 1.0, recommend abort, reason "
         "'malformed reply', unsafe - and the text is kept in raw")
def _malformed():
    with _frame() as f:
        v = _stub("the hand looks fine, go ahead").ask(f, "CP1", {})
    return _is_abort(v, "malformed reply") and v["raw"] == "the hand looks fine, go ahead", f"{v['reason']}; unsafe {v['unsafe']}"


@fixture("JUDGE_prose_wrapped_json_is_malformed",
         "a well-formed object with a sentence before it; a fence with a sentence before or after it; a fence tagged "
         "python; a fence around an object with an extra key; two objects in one fence: each malformed (ruling 8 "
         "unwraps a reply that is nothing but one ```json fence around the rubric's exact object, nothing else)")
def _prose():
    body = json.dumps(GOOD)
    replies = ["Here is my verdict: " + body, "Verdict:\n```json\n" + body + "\n```", "```json\n" + body + "\n```\nDone.",
               "```python\n" + body + "\n```", "```json\n" + json.dumps({**GOOD, "note": "x"}) + "\n```",
               "```json\n" + body + "\n" + body + "\n```"]
    with _frame() as f:
        got = [_stub(r).ask(f, "FREEZE", {}) for r in replies]
    ok = all(_is_abort(v, "malformed reply") and not v.get("fence_stripped") for v in got)
    return ok, "; ".join(v["reason"] for v in got)


@fixture("JUDGE_fenced_exact_object_accepted_and_logged",
         "ruling 8: a reply that is nothing but a ```json (or bare ```) fence around the rubric's exact object is "
         "accepted - the six fields pass through, fence_stripped is true, raw keeps the model's text with the fence; "
         "the same object unfenced says fence_stripped false", kind="control")
def _fenced():
    body = json.dumps(GOOD, indent=2)
    with _frame() as f:
        a = _stub("```json\n" + body + "\n```").ask(f, "FREEZE", {})
        b = _stub("  ```\n" + body + "\n```  ").ask(f, "FREEZE", {})
        c = _stub(json.dumps(GOOD)).ask(f, "FREEZE", {})
    ok = (all(v["reason"] == GOOD["reason"] and v["fence_stripped"] is True and v["raw"].startswith(("```", "  ```"))
              for v in (a, b)) and c["fence_stripped"] is False and c["reason"] == GOOD["reason"])
    return ok, f"json fence: {a['fence_stripped']}; bare fence: {b['fence_stripped']}; unfenced: {c['fence_stripped']}"


@fixture("JUDGE_schema_violations_are_malformed",
         "nine replies that are JSON but not the rubric's object - a missing field, an extra field, a string for a "
         "bool, 1 for a bool, p_unsafe 1.2, p_unsafe NaN, recommend 'go', an empty reason, an array: each malformed")
def _schema():
    bad = [{k: v for k, v in GOOD.items() if k != "reason"}, {**GOOD, "note": "x"}, {**GOOD, "hand_present": "true"},
           {**GOOD, "hand_open_waiting": 1}, {**GOOD, "p_unsafe": 1.2}, {**GOOD, "p_unsafe": float("nan")},
           {**GOOD, "recommend": "go"}, {**GOOD, "reason": " "}, [GOOD]]
    with _frame() as f:
        got = [_stub(json.dumps(b)).ask(f, "CP2", {}) for b in bad]
    n = sum(_is_abort(v, "malformed reply") for v in got)
    return n == len(bad), f"{n} of {len(bad)} read as malformed"


@fixture("JUDGE_timeout_returns_at_budget_abort_unsafe",
         "the stub answers after 9 s (a well-formed 'proceed'): ask() returns at the 8 s budget with p_unsafe 1.0, "
         "recommend abort, reason 'timeout' - the late reply is not read")
def _timeout():
    late = {"text": json.dumps({**GOOD, "p_unsafe": 0.0, "recommend": "proceed"}), "delay_s": 9.0}
    with _frame() as f:
        t0 = time.monotonic()
        v = _stub(late).ask(f, "CP1", {})
        took = time.monotonic() - t0
    return _is_abort(v, "timeout") and 7.9 <= took <= 8.5, f"returned after {took:.2f} s; latency_s {v['latency_s']}; {v['reason']}"


@fixture("JUDGE_p_unsafe_at_bar_is_unsafe",
         "a well-formed reply with p_unsafe 0.5 and recommend proceed: unsafe is true (the bar is >= 0.5); the "
         "model's fields are passed through unchanged")
def _at_bar():
    with _frame() as f:
        v = _stub(json.dumps({**GOOD, "p_unsafe": 0.5, "recommend": "proceed"})).ask(f, "CP1", {})
    return v["unsafe"] is True and v["p_unsafe"] == 0.5 and v["recommend"] == "proceed", f"p_unsafe {v['p_unsafe']} -> unsafe {v['unsafe']}"


@fixture("JUDGE_p_unsafe_below_bar_is_not_unsafe",
         "a well-formed reply with p_unsafe 0.49 and recommend proceed: unsafe is false", kind="control")
def _below_bar():
    with _frame() as f:
        v = _stub(json.dumps({**GOOD, "p_unsafe": 0.49, "recommend": "proceed"})).ask(f, "CP1", {})
    return v["unsafe"] is False and v["recommend"] == "proceed", f"p_unsafe {v['p_unsafe']} -> unsafe {v['unsafe']}"


@fixture("JUDGE_well_formed_reply_passes_through",
         "a well-formed reply padded with 5 kB of trailing spaces: the six fields come back as sent, with latency_s, "
         "model 'stub', the rubric's and the frame's sha256 (recomputed here from the bytes) and raw cut to 2 kB; a "
         "10 kB non-JSON reply also keeps raw <= 2 kB", kind="control")
def _pass_through():
    import hashlib
    with _frame() as f:
        v = _stub(json.dumps(GOOD) + " " * 5000).ask(f, "FREEZE", {"trial": 1})
        junk = _stub("é" * 5000).ask(f, "FREEZE", {})
        frame_sha = hashlib.sha256(f.read_bytes()).hexdigest()
    rubric_sha = hashlib.sha256(jd.RUBRIC_PATH.read_bytes()).hexdigest()
    ok = (all(v[k] == GOOD[k] for k in jd.FIELDS) and v["unsafe"] is False and v["model"] == "stub"
          and v["rubric_sha256"] == rubric_sha and v["frame_sha256"] == frame_sha
          and isinstance(v["latency_s"], float) and len(v["raw"].encode()) <= jd.RAW_MAX_BYTES
          and len(junk["raw"].encode()) <= jd.RAW_MAX_BYTES and junk["raw"])
    return bool(ok), f"raw {len(v['raw'].encode())} and {len(junk['raw'].encode())} bytes; rubric {rubric_sha[:12]}; frame {frame_sha[:12]}"


@fixture("JUDGE_rubric_edit_changes_hash",
         "a copy of the rubric with one byte changed: the verdict's rubric_sha256 differs from the unedited one's "
         "(the hash is taken from the file at every call, not cached)")
def _rubric_hash():
    with _frame() as f, tempfile.TemporaryDirectory() as td:
        copy = Path(td) / "rubric.md"
        copy.write_bytes(jd.RUBRIC_PATH.read_bytes())
        j = jd.load(provider="stub", stub_script=[json.dumps(GOOD)], rubric_path=copy)
        a = j.ask(f, "CP1", {})["rubric_sha256"]
        b0 = copy.read_bytes()
        copy.write_bytes(b0[:-1] + bytes([b0[-1] ^ 1]))
        b = j.ask(f, "CP1", {})["rubric_sha256"]
        real = _stub(json.dumps(GOOD)).ask(f, "CP1", {})["rubric_sha256"]
    return a == real and a != b, f"unedited {a[:12]}, one byte changed {b[:12]}"


# ------------------------------------------------------------------------------------------ not asked at all
@fixture("JUDGE_not_configured_is_unsafe",
         "no .env; a .env with a provider but no key; a .env naming an unknown provider: each gives UNSAFE 'judge "
         "not configured' and model 'none' - a stub is never chosen silently")
def _not_configured():
    with _frame() as f, tempfile.TemporaryDirectory() as td:
        t = Path(td)
        (t / "nokey.env").write_text("JUDGE_PROVIDER=anthropic\n")
        (t / "other.env").write_text("JUDGE_PROVIDER=llama\nANTHROPIC_API_KEY=" + _fake_key() + "\n")
        js = [jd.load(env_path=t / "absent.env"), jd.load(env_path=t / "nokey.env"), jd.load(env_path=t / "other.env")]
        got = [j.ask(f, "CP1", {}) for j in js]
    ok = all(_is_abort(v, "judge not configured") and v["model"] == "none" for v in got) and not any(j.configured for j in js)
    return ok, "; ".join(str(j.problem) for j in js)


@fixture("JUDGE_explicit_stub_says_stub",
         "load(provider='stub') with no script: each phase gets a well-formed reply whose model is 'stub' and whose "
         "reason says no model looked at the frame", kind="control")
def _stub_default():
    with _frame() as f:
        got = {ph: jd.load(provider="stub").ask(f, ph, {}) for ph in jd.PHASES}
    ok = all(v["model"] == "stub" and v["reason"] == pv.STUB_REASON and v["unsafe"] is False for v in got.values())
    return ok, "; ".join(f"{ph}: {v['recommend']}" for ph, v in got.items())


@fixture("JUDGE_bad_frame_or_phase_is_unsafe",
         "a frame path that does not exist, a file that is neither PNG nor JPEG, a phase outside CP1/CP2/CP3/FREEZE: "
         "each UNSAFE, and the stub is not called")
def _bad_inputs():
    j = _stub(json.dumps(GOOD))
    with _frame() as f, _frame(b"not an image", "frame.txt") as g:
        a = j.ask(f.parent / "missing.png", "CP1", {})
        b = j.ask(g, "CP1", {})
        c = j.ask(f, "CP9", {})
    ok = (_is_abort(a, "frame not readable") and a["frame_sha256"] is None and _is_abort(b, "frame not readable")
          and _is_abort(c, "unknown phase") and j.provider.calls == 0)
    return ok, f"{a['reason']}; {b['reason']}; {c['reason']}; provider calls {j.provider.calls}"


# ------------------------------------------------------------------------------------------ the providers
def _env_file(td: Path, provider: str, model: str | None) -> Path:
    key_name = "ANTHROPIC_API_KEY" if provider == "anthropic" else "OPENAI_API_KEY"
    p = td / f"{provider}.env"
    p.write_text(f"JUDGE_PROVIDER={provider}\n" + (f"JUDGE_MODEL={model}\n" if model else "")
                 + f'export {key_name}="{_fake_key()}"   # a fake\n', encoding="utf-8")
    return p


def _anthropic_models() -> list:
    cap = lambda vision, effort: {"image_input": {"supported": vision}, "effort": {"supported": effort, "low": {"supported": effort}}}
    return [{"id": "claude-opus-5-5", "created_at": "2026-08-20T00:00:00Z", "capabilities": cap(True, True)},
            {"id": "claude-sonnet-5-5", "created_at": "2026-09-01T00:00:00Z", "capabilities": cap(True, True)},
            {"id": "claude-haiku-4-5", "created_at": "2025-10-15T00:00:00Z", "capabilities": cap(True, False)},
            {"id": "claude-haiku-0-0", "created_at": "2024-01-01T00:00:00Z", "capabilities": cap(True, False)},
            {"id": "claude-haiku-9-text", "created_at": "2027-01-01T00:00:00Z", "capabilities": cap(False, False)}]


@fixture("JUDGE_anthropic_request_and_model_choice",
         "the Anthropic provider behind a mock transport, JUDGE_MODEL unset: the models are listed ONCE at load, the "
         "choice is the newest vision-capable haiku (the rule in the code), and each ask sends one POST /v1/messages "
         "with the key in x-api-key, anthropic-version 2023-06-01, the rubric as system, a base64 image block and the "
         "phase + context as text; no effort setting on a model whose capabilities lack it", kind="control")
def _anthropic():
    seen = []

    def handler(req: httpx.Request) -> httpx.Response:
        seen.append(req)
        if req.method == "GET":
            return httpx.Response(200, json={"data": _anthropic_models(), "has_more": False, "last_id": "x"})
        return httpx.Response(200, json={"content": [{"type": "text", "text": json.dumps(GOOD)}], "stop_reason": "end_turn"})
    with _frame() as f, tempfile.TemporaryDirectory() as td:
        j = jd.load(env_path=_env_file(Path(td), "anthropic", None), transport=httpx.MockTransport(handler))
        v1, v2 = j.ask(f, "FREEZE", {"trial": 3}), j.ask(f, "CP3", {})
    gets = [r for r in seen if r.method == "GET"]
    posts = [r for r in seen if r.method == "POST"]
    body = json.loads(posts[0].content) if posts else {}
    content = (body.get("messages") or [{}])[0].get("content") or [{}, {}]
    ok = (len(gets) == 1 and len(posts) == 2 and j.model == "claude-haiku-4-5" and v1["model"] == "claude-haiku-4-5"
          and v1["unsafe"] is False and v2["recommend"] == "handover"
          and posts[0].url.path == "/v1/messages" and posts[0].headers["x-api-key"] == _fake_key()
          and posts[0].headers["anthropic-version"] == "2023-06-01"
          and body["system"] == jd.RUBRIC_PATH.read_text(encoding="utf-8") and body["model"] == "claude-haiku-4-5"
          and content[0]["type"] == "image" and content[0]["source"]["media_type"] == "image/png"
          and base64.b64decode(content[0]["source"]["data"]) == PNG_1PX
          and content[1]["text"].startswith("phase: FREEZE\ncontext: {\"trial\": 3}")
          and "output_config" not in body and j.describe()["model_rule"] == pv.ANTHROPIC_RULE)
    return ok, f"listings {len(gets)}, messages {len(posts)}, chosen {j.model}"


@fixture("JUDGE_anthropic_named_model_low_effort",
         "JUDGE_MODEL names a model whose capabilities list effort.low: no listing; one capability read at load; the "
         "request carries output_config.effort 'low' (the budget is 8 s)", kind="control")
def _anthropic_named():
    seen = []

    def handler(req: httpx.Request) -> httpx.Response:
        seen.append(req)
        if req.method == "GET":
            return httpx.Response(200, json=_anthropic_models()[0])
        return httpx.Response(200, json={"content": [{"type": "thinking", "thinking": ""},
                                                     {"type": "text", "text": json.dumps(GOOD)}]})
    with _frame() as f, tempfile.TemporaryDirectory() as td:
        j = jd.load(env_path=_env_file(Path(td), "anthropic", "claude-opus-5-5"), transport=httpx.MockTransport(handler))
        v = j.ask(f, "CP1", {})
    gets = [r.url.path for r in seen if r.method == "GET"]
    body = json.loads(next(r for r in seen if r.method == "POST").content)
    ok = (gets == ["/v1/models/claude-opus-5-5"] and body.get("output_config") == {"effort": "low"}
          and v["model"] == "claude-opus-5-5" and v["unsafe"] is False and j.describe()["model_source"] == "JUDGE_MODEL")
    return ok, f"GETs {gets}; output_config {body.get('output_config')}"


@fixture("JUDGE_openai_request_and_model_choice",
         "the OpenAI provider behind a mock transport, JUDGE_MODEL unset: one listing at load, the choice is the "
         "newest 'nano' chat model (audio / tts / embedding / gpt-3 ids are passed over), and the ask is one POST "
         "/v1/chat/completions with a Bearer header, the rubric as the system message and the frame as a data URL",
         kind="control")
def _openai():
    seen = []
    models = [{"id": "gpt-4o-mini", "created": 1721000000}, {"id": "gpt-4.1-nano", "created": 1744000000},
              {"id": "gpt-5-nano", "created": 1754000000}, {"id": "gpt-5-nano-tts", "created": 1790000000},
              {"id": "gpt-3.5-turbo", "created": 1677000000}, {"id": "text-embedding-3-small", "created": 1705000000},
              {"id": "gpt-4o-mini-audio-preview", "created": 1734000000}, {"id": "gpt-5", "created": 1754000001}]

    def handler(req: httpx.Request) -> httpx.Response:
        seen.append(req)
        if req.method == "GET":
            return httpx.Response(200, json={"object": "list", "data": models})
        return httpx.Response(200, json={"choices": [{"message": {"role": "assistant", "content": json.dumps(GOOD)}}]})
    with _frame(b"\xff\xd8\xff\xe0 jpeg-ish", "frame.jpg") as f, tempfile.TemporaryDirectory() as td:
        j = jd.load(env_path=_env_file(Path(td), "openai", None), transport=httpx.MockTransport(handler))
        v = j.ask(f, "CP2", {})
    posts = [r for r in seen if r.method == "POST"]
    body = json.loads(posts[0].content) if posts else {}
    msgs = body.get("messages") or [{}, {"content": [{}, {"image_url": {}}]}]
    ok = (sum(r.method == "GET" for r in seen) == 1 and len(posts) == 1 and j.model == "gpt-5-nano"
          and posts[0].url.path == "/v1/chat/completions" and posts[0].headers["authorization"] == "Bearer " + _fake_key()
          and msgs[0] == {"role": "system", "content": jd.RUBRIC_PATH.read_text(encoding="utf-8")}
          and msgs[1]["content"][1]["image_url"]["url"].startswith("data:image/jpeg;base64,")
          and v["unsafe"] is False and v["model"] == "gpt-5-nano" and j.describe()["model_rule"] == pv.OPENAI_RULE)
    return ok, f"chosen {j.model}; requests {[r.method for r in seen]}"


@fixture("JUDGE_provider_failure_is_unsafe",
         "HTTP 500 from the provider, an empty content list (a refusal), and a listing that fails at load: UNSAFE "
         "'provider error', UNSAFE 'malformed reply', UNSAFE 'judge not configured'")
def _provider_failure():
    def h500(req):
        return httpx.Response(500, json={"type": "error", "error": {"type": "api_error", "message": "overloaded"}})

    def refusal(req):
        return httpx.Response(200, json={"content": [], "stop_reason": "refusal"})
    with _frame() as f, tempfile.TemporaryDirectory() as td:
        env = _env_file(Path(td), "anthropic", "claude-haiku-4-5")
        a = jd.load(env_path=env, transport=httpx.MockTransport(h500)).ask(f, "CP1", {})
        b = jd.load(env_path=env, transport=httpx.MockTransport(refusal)).ask(f, "CP1", {})
        j = jd.load(env_path=_env_file(Path(td), "openai", None), transport=httpx.MockTransport(h500))
        c = j.ask(f, "CP1", {})
    ok = (_is_abort(a, "provider error") and a["raw"] == "anthropic: http 500 api_error" and _is_abort(b, "malformed reply")
          and _is_abort(c, "judge not configured") and not j.configured)
    return ok, f"{a['reason']} ({a['raw']}); {b['reason']}; {c['reason']} ({j.problem})"


# ------------------------------------------------------------------------------------------ secrets
@fixture("JUDGE_fake_key_never_leaves",
         "a fake key in the .env; the provider answers 401 with the key quoted in its body (as OpenAI does), then "
         "the connection fails with the key in the exception text, then the model's own reply quotes the key: the key "
         "is in no verdict, no exception text or repr, no log record, nothing on stdout or stderr, and not in the "
         "judge's or the env's repr")
def _key_never_leaves():
    key = _fake_key()

    def echo401(req):
        return httpx.Response(401, json={"error": {"type": "invalid_request_error", "code": "invalid_api_key",
                                                   "message": f"Incorrect API key provided: {key}"}})

    def boom(req):
        raise httpx.ConnectError(f"cannot connect with header x-api-key: {key}")

    def parrot(req):
        text = json.dumps({**GOOD, "reason": f"the key is {key}"})
        return httpx.Response(200, json={"choices": [{"message": {"content": text}}]})
    out, err, logs = io.StringIO(), io.StringIO(), io.StringIO()
    handler = logging.StreamHandler(logs)
    root = logging.getLogger()
    old_level = root.level
    root.addHandler(handler)
    root.setLevel(logging.DEBUG)
    shown = []
    try:
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err), _frame() as f, \
                tempfile.TemporaryDirectory() as td:
            env_path = _env_file(Path(td), "openai", "gpt-5-nano")
            for h in (echo401, boom, parrot):
                j = jd.load(env_path=env_path, transport=httpx.MockTransport(h))
                shown += [json.dumps(j.ask(f, "FREEZE", {"note": f"context with {key}"})), repr(j), repr(j.provider),
                          json.dumps(j.describe())]
                try:
                    j.provider.complete("s", "phase: CP1", PNG_1PX, "image/png", 2.0)
                except pv.ProviderError as e:
                    shown += [str(e), repr(e), repr(e.__context__), repr(e.__cause__), repr(e.args)]
                print(j, j.describe())
            env = load_env(env_path)
            shown += [repr(env), str(env), f"{env}", "%s" % env, json.dumps(env.names_set())]
            unlisted = jd.load(env_path=_env_file(Path(td), "openai", None), transport=httpx.MockTransport(echo401))
            shown += [str(unlisted.problem), json.dumps(unlisted.describe())]
    finally:
        root.removeHandler(handler)
        root.setLevel(old_level)
    blob = "\n".join(shown) + out.getvalue() + err.getvalue() + logs.getvalue()
    sent_ok = env.value("OPENAI_API_KEY") == key            # control: the key was really there to leak
    return sent_ok and key not in blob and key[:12] not in blob, f"{len(blob)} characters inspected; key present: {key in blob}"


@fixture("JUDGE_env_parser",
         "a .env text with a comment, a blank line, `export`, double and single quotes, an inline comment, and a "
         "line that is not KEY=VALUE: the four names parse to their values; repr / str / format / names_set show "
         "names only; pickling is refused", kind="control")
def _env_parser():
    import pickle
    key = _fake_key()
    text = (f"# judge\n\nJUDGE_PROVIDER = anthropic   # which\nexport JUDGE_MODEL='claude-haiku-4-5'\n"
            f'ANTHROPIC_API_KEY="{key}"\nthis line is not an assignment\nOPENAI_API_KEY=\n')
    d = parse(text)
    env = Env(d)
    try:
        pickle.dumps(env)
        pickled = True
    except TypeError:
        pickled = False
    ok = (d == {"JUDGE_PROVIDER": "anthropic", "JUDGE_MODEL": "claude-haiku-4-5", "ANTHROPIC_API_KEY": key, "OPENAI_API_KEY": ""}
          and env.names_set() == {"JUDGE_PROVIDER": True, "JUDGE_MODEL": True, "ANTHROPIC_API_KEY": True, "OPENAI_API_KEY": False}
          and env.secrets() == (key,) and key not in repr(env) + str(env) + f"{env}" and "haiku" not in repr(env)
          and not pickled and redact({"a": [f"x {key} y"]}, ()) == {"a": ["x [REDACTED] y"]})
    return ok, f"{repr(env)}; picklable {pickled}"


# ------------------------------------------------------------------------------------------ what it imports
@fixture("JUDGE_import_graph",
         "every module in judge/ (tests aside) imports only the standard library, httpx and judge itself (read from "
         "the source), and a fresh interpreter that imports judge.judge has loaded nothing from anchor/, the runner, "
         "the planner, the monitor or common/")
def _import_graph():
    allowed = set(sys.stdlib_module_names) | {"httpx", "judge"}
    bad = []
    for p in sorted(HERE.glob("*.py")):
        for node in ast.walk(ast.parse(p.read_text(encoding="utf-8"))):
            if isinstance(node, ast.Import):
                bad += [f"{p.name}: {a.name}" for a in node.names if a.name.split(".")[0] not in allowed]
            elif isinstance(node, ast.ImportFrom) and node.level == 0:
                if (node.module or "").split(".")[0] not in allowed:
                    bad.append(f"{p.name}: {node.module}")
    code = ("import sys; sys.dont_write_bytecode = True; sys.path.insert(0, %r); import judge.judge; "
            "names = ('g3', 'safety', 'fk', 'null_session', 'null_plan', 'run_anchor', 'fake_bus', 'common', 'planner', "
            "'monitor', 'handover_session', 'so101_sim'); "
            "print(sorted(m for m in sys.modules if m.split('.')[0] in names))") % str(HERE.parent)
    r = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=60)
    loaded = r.stdout.strip()
    return (not bad) and r.returncode == 0 and loaded == "[]", f"source: {bad or 'clean'}; loaded at run time: {loaded or r.stderr[-200:]}"


def run_all(names: list | None = None) -> list:
    """-> [{"name", "kind", "what", "ok", "detail"}] (a fixture that raises is not ok)."""
    rows = []
    for fx in REGISTRY:
        if names and fx["name"] not in names:
            continue
        try:
            ok, detail = fx["fn"]()
        except Exception as e:  # noqa: BLE001
            ok, detail = False, f"raised {type(e).__name__}: {e}"
        rows.append({"name": fx["name"], "kind": fx["kind"], "what": fx["what"], "ok": bool(ok), "detail": detail})
    return rows


def main() -> int:
    rows = run_all(sys.argv[1:] or None)
    for r in rows:
        print(f"  [{'ok ' if r['ok'] else 'BAD'}] {r['name']:46} ({r['kind']:8}) {r['detail']}")
    print("JUDGE FIXTURES: " + ("ALL AS EXPECTED" if all(r["ok"] for r in rows) else "NOT AS EXPECTED"))
    return 0 if all(r["ok"] for r in rows) else 1


if __name__ == "__main__":
    sys.exit(main())

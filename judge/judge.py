"""The checkpoint judge: a vision model reads one still frame while the arm is stationary, and recommends.

    from judge import judge
    j = judge.load()                              # provider from .env (JUDGE_PROVIDER, JUDGE_MODEL, the key)
    j = judge.load(provider="stub")               # fixtures and dry runs: no network; every verdict says model "stub"
    verdict = j.ask(frame_path, "CP1", {"trial": 1})
    judge.configure(j); judge.ask(frame_path, phase, context)      # the module-level form (PLAN.md section 5)

It recommends; it never commands, and code rules are the only hard gate (the operator's rule 5). A verdict is the
rubric's six fields plus latency_s, model, rubric_sha256, frame_sha256, raw (the model's text, <= 2 kB) and unsafe -
the code's reading: true on a timeout, a malformed reply, any failure to ask, or p_unsafe >= 0.5.
  - budget 8 s wall clock from the call: the provider runs in a worker thread and ask() returns at the budget whatever
    the socket does -> p_unsafe 1.0, recommend "abort", reason "timeout";
  - the reply must be exactly one JSON object with exactly the six keys, right types, p_unsafe in 0..1, recommend one
    of the five words; prose around it, a missing or extra key -> reason "malformed reply". One exception
    (reviewer 2026-10-01, ruling 8, after the chosen model fenced 15 of 15 replies): a reply that is nothing but
    a ```json (or ```) fence around one object is unwrapped, the object must still pass the same check, and the
    verdict says fence_stripped: true; prose or anything else outside the fence stays malformed;
  - no .env, no key, an unknown provider, a failed model listing -> reason "judge not configured" (a stub is never
    chosen silently); an HTTP or network failure -> "provider error"; an unreadable frame -> "frame not readable".
ask() does not raise. The rubric's sha256 is taken from the file's bytes at every call, the frame's from the file.
This package imports the standard library, httpx and itself - nothing from the runner, the planner, the monitor or
anchor/ - and is never called inside the control loop.
"""
from __future__ import annotations

import hashlib
import json
import logging
import re
import threading
import time
from pathlib import Path

from . import providers as pv
from .env import ENV_PATH, KEY_OF, Env, load_env
from .scrub import redact

log = logging.getLogger("khv.judge")

BUDGET_S = 8.0
P_UNSAFE_BAR = 0.5
RAW_MAX_BYTES = 2048
REASON_MAX_CHARS = 500
PHASES = ("CP1", "CP2", "CP3", "FREEZE")
RECOMMEND = ("proceed", "wait", "handover", "reorient", "abort")
RUBRIC_PATH = Path(__file__).resolve().parent / "rubric.md"
HANDOVER_RUBRIC_PATH = Path(__file__).resolve().parent / "rubric_handover.md"
PLACEMENT_RUBRIC_PATH = Path(__file__).resolve().parent / "rubric_placement.md"
FIELDS = ("hand_present", "hand_open_waiting", "blade_toward_hand", "p_unsafe", "recommend", "reason")


def rubric_kind(phase: str, context: dict | None) -> str:
    """Which rubric a call is asked under (the operator, 2026-10-03): 'handover' (rubric_handover.md) for CP3 and for
    the handover freeze - FREEZE with the prop held and a hand seen by the monitor; 'placement' (rubric_placement.md)
    for those same two when the context says mode 'placement' (--palm-placement); 'checkpoint' (rubric.md, unchanged)
    for every other call: CP1, CP2, any other freeze. Read from the runner's context only."""
    c = context if isinstance(context, dict) else {}
    try:
        hands = int(c.get("hands_seen_by_monitor") or 0)
    except (TypeError, ValueError):
        hands = 0
    if c.get("mode") == "placement" and (phase == "CP3" or (phase == "FREEZE" and c.get("holding_prop") is True)):
        return "placement"                         # the hand count may read 0 for a frame: the question is the same
    if phase == "CP3" or (phase == "FREEZE" and c.get("holding_prop") is True and hands >= 1):
        return "handover"
    return "checkpoint"


def _sha(b: bytes) -> str:
    return hashlib.sha256(b).hexdigest()


def _media_type(b: bytes) -> str | None:
    if b[:8] == b"\x89PNG\r\n\x1a\n":
        return "image/png"
    if b[:2] == b"\xff\xd8":
        return "image/jpeg"
    return None


def _clip(text: str) -> str:
    return text.encode("utf-8")[:RAW_MAX_BYTES].decode("utf-8", errors="ignore")


_FENCE = re.compile(r"\A```(?:json)?[ \t]*\n(?P<body>.*)\n[ \t]*```\Z", re.DOTALL)


def unfence(text: str) -> tuple:
    """(the text to parse, whether a fence was stripped). Only a reply that is nothing but one ```json / ``` fence
    is unwrapped; anything else is returned as it came (and then fails to parse if it is not an object)."""
    s = text.strip() if isinstance(text, str) else ""
    m = _FENCE.match(s)
    return (m.group("body"), True) if m else (s, False)


def parse_reply_fenced(text: str) -> tuple:
    """(the six fields or None, fence_stripped)."""
    body, fenced = unfence(text)
    fields = parse_reply(body)
    return fields, bool(fenced and fields is not None)


def parse_reply(text: str) -> dict | None:
    """The model's text -> the six rubric fields, or None (malformed). Strict: see the module docstring."""
    try:
        d = json.loads(text.strip())
    except (ValueError, AttributeError):
        return None
    if not isinstance(d, dict) or set(d) != set(FIELDS):
        return None
    p = d["p_unsafe"]
    ok = (isinstance(d["hand_present"], bool) and isinstance(d["hand_open_waiting"], bool)
          and (d["blade_toward_hand"] is None or isinstance(d["blade_toward_hand"], bool))
          and isinstance(p, (int, float)) and not isinstance(p, bool) and 0.0 <= p <= 1.0
          and d["recommend"] in RECOMMEND
          and isinstance(d["reason"], str) and 0 < len(d["reason"].strip()) <= REASON_MAX_CHARS)
    return {k: d[k] for k in FIELDS} if ok else None


class Judge:
    def __init__(self, provider: pv.Provider | None, rubric_path: Path = RUBRIC_PATH, budget_s: float = BUDGET_S,
                 secrets: tuple = (), choice: dict | None = None, problem: str | None = None,
                 handover_rubric_path: Path = HANDOVER_RUBRIC_PATH, placement_rubric_path: Path = PLACEMENT_RUBRIC_PATH):
        self.provider, self.rubric_path, self.budget_s = provider, Path(rubric_path), float(budget_s)
        self.rubric_paths = {"checkpoint": self.rubric_path, "handover": Path(handover_rubric_path),
                             "placement": Path(placement_rubric_path)}
        self._secrets = tuple(secrets)
        self.choice = choice or {}
        self.problem = problem                       # why it is not configured (names and fixed words only)

    @property
    def configured(self) -> bool:
        return self.provider is not None

    @property
    def model(self) -> str:
        return self.provider.model if self.provider is not None else "none"

    def describe(self) -> dict:
        """For the session-start row: what will judge, chosen how. No values from the .env."""
        try:
            rubric_sha = _sha(self.rubric_path.read_bytes())
        except OSError:
            rubric_sha = None
        try:
            handover_sha = _sha(self.rubric_paths["handover"].read_bytes())
            placement_sha = _sha(self.rubric_paths["placement"].read_bytes())
        except OSError:
            handover_sha = placement_sha = None
        return redact({"rubric_handover_sha256": handover_sha, "rubric_placement_sha256": placement_sha, "configured": self.configured, "provider": getattr(self.provider, "name", None),
                       "model": self.model, "model_source": self.choice.get("source"),
                       "model_rule": self.choice.get("rule"), "models_listed": self.choice.get("listed"),
                       "problem": self.problem, "rubric_sha256": rubric_sha, "budget_s": self.budget_s,
                       "p_unsafe_bar": P_UNSAFE_BAR}, self._secrets)

    def _verdict(self, fields: dict, t0: float, rubric_sha, frame_sha, raw: str, unsafe: bool) -> dict:
        v = {**fields, "latency_s": round(time.monotonic() - t0, 3), "model": self.model, "rubric_sha256": rubric_sha,
             "frame_sha256": frame_sha, "raw": _clip(raw), "unsafe": bool(unsafe)}
        return redact(v, self._secrets)

    def _fail(self, reason: str, t0: float, rubric_sha, frame_sha, raw: str = "") -> dict:
        fields = {"hand_present": None, "hand_open_waiting": None, "blade_toward_hand": None, "p_unsafe": 1.0,
                  "recommend": "abort", "reason": reason}
        return self._verdict(fields, t0, rubric_sha, frame_sha, raw, True)

    def ask(self, frame_path, phase: str, context: dict | None = None) -> dict:
        """One call -> the verdict, with the rubric it was asked under ('rubric') and that file's sha256."""
        kind = rubric_kind(phase, context)
        return {**self._ask(frame_path, phase, context, self.rubric_paths[kind]), "rubric": kind}

    def _ask(self, frame_path, phase: str, context: dict | None, rubric_path: Path) -> dict:
        t0 = time.monotonic()
        rubric_sha = frame_sha = None
        try:
            rubric = rubric_path.read_bytes()
            rubric_sha = _sha(rubric)
        except OSError:
            return self._fail("rubric not readable", t0, None, None)
        try:
            frame = Path(frame_path).read_bytes()
            frame_sha = _sha(frame)
        except (OSError, TypeError):
            return self._fail("frame not readable", t0, rubric_sha, None)
        if self.provider is None:
            return self._fail("judge not configured", t0, rubric_sha, frame_sha)
        if phase not in PHASES:
            return self._fail("unknown phase", t0, rubric_sha, frame_sha)
        media = _media_type(frame)
        if media is None:
            return self._fail("frame not readable", t0, rubric_sha, frame_sha, "the frame is neither PNG nor JPEG")
        try:
            ctx = json.dumps(context or {}, sort_keys=True, default=str)
        except (TypeError, ValueError):
            ctx = "{}"
        user = f"phase: {phase}\ncontext: {redact(ctx, self._secrets)}\nReply with the JSON object only."
        box: dict = {}
        system = rubric.decode("utf-8", errors="replace")

        def work() -> None:
            try:
                box["text"] = self.provider.complete(system, user, frame, media, max(0.1, self.budget_s))
            except pv.ProviderError as e:
                box["error"] = str(e)
            except Exception as e:  # noqa: BLE001 - never the exception's own text: its class name only
                box["error"] = f"judge error: {type(e).__name__}"

        worker = threading.Thread(target=work, name="khv-judge", daemon=True)
        worker.start()
        worker.join(max(0.0, self.budget_s - (time.monotonic() - t0)))
        got = dict(box)                               # a reply that lands after the budget is not read
        if worker.is_alive() or not got:
            return self._fail("timeout", t0, rubric_sha, frame_sha)
        if "error" in got:
            return self._fail("provider error", t0, rubric_sha, frame_sha, got["error"])
        text = got.get("text") if isinstance(got.get("text"), str) else ""
        fields, fenced = parse_reply_fenced(text)
        if fields is None:
            return self._fail("malformed reply", t0, rubric_sha, frame_sha, text)
        if fenced:
            log.info("judge: a code fence around the reply was stripped (ruling 8); the object passed the schema")
        v = self._verdict(fields, t0, rubric_sha, frame_sha, text, fields["p_unsafe"] >= P_UNSAFE_BAR)
        return {**v, "fence_stripped": fenced}


def _not_configured(problem: str, rubric_path: Path, budget_s: float, env: Env) -> Judge:
    log.warning("judge not configured: %s", problem)
    return Judge(None, rubric_path, budget_s, env.secrets(), problem=problem)


def load(provider: str | None = None, env_path: Path | None = None, rubric_path: Path = RUBRIC_PATH,
         stub_script=None, budget_s: float = BUDGET_S, transport=None) -> Judge:
    """The judge for this run. provider="stub" must be asked for; otherwise the .env decides, and anything missing
    gives a judge whose every verdict is UNSAFE "judge not configured". The models are listed here, once, when
    JUDGE_MODEL is unset. `transport` is an httpx transport for fixtures."""
    if provider == "stub":
        return Judge(pv.StubProvider(stub_script), rubric_path, budget_s,
                     choice={"source": "asked for explicitly (provider='stub')", "rule": None})
    env = load_env(env_path if env_path is not None else ENV_PATH)
    name = (provider or env.value("JUDGE_PROVIDER") or "").strip().lower()
    if name not in KEY_OF:
        return _not_configured("JUDGE_PROVIDER is not set to anthropic or openai", rubric_path, budget_s, env)
    key = env.value(KEY_OF[name])
    if key is None:
        return _not_configured(f"{KEY_OF[name]} is not set", rubric_path, budget_s, env)
    model = env.value("JUDGE_MODEL")
    choice = {"source": "JUDGE_MODEL", "rule": None}
    info: dict = {}
    if model is None:
        problem = None
        try:
            if name == "anthropic":
                listed = pv.anthropic_models(key, budget_s, transport)
                info, rule = pv.choose_anthropic(listed), pv.ANTHROPIC_RULE
            else:
                listed = pv.openai_models(key, budget_s, transport)
                info, rule = pv.choose_openai(listed), pv.OPENAI_RULE
        except pv.ProviderError as e:
            problem = f"the model listing failed ({e})"
        if problem is not None:
            return _not_configured(problem, rubric_path, budget_s, env)
        model = info["id"]
        choice = {"source": "listed once at start", "rule": rule, "listed": len(listed)}
        log.info("judge model chosen: %s (%s; %d listed)", model, rule, len(listed))
    if name == "anthropic":
        prov = (pv.AnthropicProvider(key, model, pv._supported(info, "effort", "low"), transport) if info
                else pv.AnthropicProvider.with_capabilities(key, model, budget_s, transport))
    else:
        prov = pv.OpenAIProvider(key, model, transport)
    return Judge(prov, rubric_path, budget_s, env.secrets(), choice=choice)


_default: Judge | None = None


def configure(judge: Judge | None) -> None:
    """Set (or clear) the judge the module-level ask() uses."""
    global _default
    _default = judge


def ask(frame_path, phase: str, context: dict | None = None) -> dict:
    """PLAN.md section 5: judge.ask(frame_path, phase, context) -> verdict. Loads from the .env on first use."""
    global _default
    if _default is None:
        _default = load()
    return _default.ask(frame_path, phase, context)

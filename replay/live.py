#!/usr/bin/env python
"""The live view (the operator, 2026-10-04): replay.html rebuilt every few seconds from the newest arm session while it
runs, the page reloading itself - beside the terminals, what the runner is doing. Reads the chain only; never writes it.

    python replay/live.py [--every 3] [--session sessions/KH-S...]      (Ctrl-C to stop; the page stays)
"""
from __future__ import annotations

import argparse
import sys
import time
import webbrowser
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
import build_replay  # noqa: E402

from common import paths  # noqa: E402

REFRESH = '<meta http-equiv="refresh" content="{s}">'


def newest(sessions: Path) -> Path | None:
    ds = [d for d in sessions.glob("KH-S*") if (d / "session.jsonl").is_file()]
    return max(ds, key=lambda d: (d / "session.jsonl").stat().st_mtime) if ds else None


def main(argv: list | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--every", type=float, default=3.0)
    ap.add_argument("--session", default=None)
    ap.add_argument("--sessions-dir", default=str(paths.SESSIONS))
    ap.add_argument("--no-browser", action="store_true")
    a = ap.parse_args(argv)
    opened, last_err = None, None
    print(f"live view: rebuilding every {a.every:g} s (Ctrl-C to stop)", flush=True)
    try:
        while True:
            d = Path(a.session) if a.session else newest(Path(a.sessions_dir))
            if d is not None:
                try:
                    out = build_replay.build(d)
                    html = out.read_text(encoding="utf-8")
                    if "http-equiv=\"refresh\"" not in html:
                        html = html.replace("<head>", "<head>" + REFRESH.format(s=max(1, round(a.every))), 1)
                        out.write_text(html, encoding="utf-8")
                    if opened != d:
                        print(f"  {d.name} -> {out}", flush=True)
                        if not a.no_browser:
                            webbrowser.open(out.as_uri())
                        opened = d
                    last_err = None
                except (SystemExit, Exception) as e:  # noqa: BLE001 - a half-written last row: the next tick
                    msg = f"{type(e).__name__}: {e}"
                    if msg != last_err:
                        print(f"  waiting ({msg[:120]})", flush=True)
                        last_err = msg
            time.sleep(a.every)
    except KeyboardInterrupt:
        print("live view stopped", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())

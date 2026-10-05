#!/usr/bin/env python
"""Secrets scan over what is about to be committed. Run before every commit (reviewer 2026-10-01, ruling 7).

    python secrets_scan.py            # scans `git diff --cached` (staged changes) and the staged file names
    python secrets_scan.py --tree     # every file that would ship (tracked + untracked, not ignored) and every blob
                                      # in the git history; plus the .env's own values, searched for and never printed

A pattern scan, not gitleaks: the token shapes of anchor/g3.py's scrub (HF, sk-, AWS, GitHub, private keys) plus the
Anthropic / OpenAI key prefixes and KEY=value assignments to names that look like secrets. Exit 1 on any hit, printing
the file and line number only - never the matched text. Staged files named .env or .env.* (other than .env.example)
are a hit by name. Binary files are skipped (their names are still checked).
"""
from __future__ import annotations

import re
import subprocess
import sys

PATTERNS = {
    "huggingface token": r"hf_[A-Za-z0-9]{16,}",
    "sk- key (Anthropic / OpenAI / other)": r"sk-[A-Za-z0-9_\-]{16,}",
    "AWS access key": r"AKIA[0-9A-Z]{16}",
    "GitHub token": r"gh[pousr]_[A-Za-z0-9]{20,}",
    "private key block": r"-----BEGIN [A-Z ]*PRIVATE KEY-----",
    "secret-looking assignment": r"(?i)\b[A-Z0-9_]*(API_KEY|SECRET|TOKEN|PASSWORD)[A-Z0-9_]*\s*[=:]\s*['\"]?[A-Za-z0-9_\-]{12,}",
}
ALLOWED_ENV_NAMES = {".env.example"}


def staged_names() -> list:
    out = subprocess.run(["git", "diff", "--cached", "--name-only", "--diff-filter=ACMR"], capture_output=True, text=True,
                         check=True).stdout
    return [x for x in out.splitlines() if x.strip()]


def scan() -> list:
    hits = []
    for name in staged_names():
        base = name.rsplit("/", 1)[-1]
        if (base == ".env" or base.startswith(".env.")) and base not in ALLOWED_ENV_NAMES:
            hits.append((name, 0, "an .env file is staged"))
    diff = subprocess.run(["git", "diff", "--cached", "-U0", "--no-color", "--text"], capture_output=True, text=True,
                          errors="replace", check=True).stdout
    path, line_no = None, 0
    for line in diff.splitlines():
        if line.startswith("+++ "):
            path = line[6:] if line.startswith("+++ b/") else line[4:]
            continue
        m = re.match(r"@@ -\d+(?:,\d+)? \+(\d+)", line)
        if m:
            line_no = int(m.group(1))
            continue
        if line.startswith("+") and not line.startswith("+++"):
            for what, rx in PATTERNS.items():
                if re.search(rx, line[1:]):
                    hits.append((path, line_no, what))
            line_no += 1
    return hits


def env_values() -> list:
    """The values in .env of names that look like secrets - held in memory to search for, never printed."""
    try:
        text = open(".env", encoding="utf-8").read()
    except OSError:
        return []
    vals = []
    for line in text.splitlines():
        m = re.match(r"\s*(?:export\s+)?([A-Za-z0-9_]+)\s*=\s*['\"]?([^'\"#\s]+)", line)
        if m and re.search(r"(?i)KEY|SECRET|TOKEN|PASSWORD", m.group(1)) and len(m.group(2)) >= 12:
            vals.append(m.group(2))
    return vals


def scan_text(name: str, text: str, vals: list) -> list:
    hits = []
    for n, line in enumerate(text.splitlines(), 1):
        for what, rx in PATTERNS.items():
            if re.search(rx, line):
                hits.append((name, n, what))
        if any(v in line or v[:16] in line or v[-16:] in line for v in vals):
            hits.append((name, n, "a value from .env (or 16 characters of one)"))
    return hits


def scan_tree() -> tuple:
    """-> (files scanned, history blobs scanned, hits). Text is read as UTF-8 with replacement (binary files too)."""
    vals, hits = env_values(), []
    names = subprocess.run(["git", "ls-files", "-co", "--exclude-standard"], capture_output=True, text=True,
                           check=True).stdout.splitlines()
    for name in names:
        base = name.rsplit("/", 1)[-1]
        if (base == ".env" or base.startswith(".env.")) and base not in ALLOWED_ENV_NAMES:
            hits.append((name, 0, "an .env file would ship"))
        try:
            text = open(name, "rb").read().decode("utf-8", "replace")
        except OSError:
            continue
        hits += scan_text(name, text, vals)
    objs = subprocess.run(["git", "rev-list", "--objects", "--all"], capture_output=True, text=True,
                          check=True).stdout.splitlines()
    blobs = {}
    for line in objs:
        sha, _, path = line.partition(" ")
        if path:
            blobs.setdefault(sha, path)
    n_blobs = 0
    for sha, path in blobs.items():
        kind = subprocess.run(["git", "cat-file", "-t", sha], capture_output=True, text=True).stdout.strip()
        if kind != "blob":
            continue
        n_blobs += 1
        data = subprocess.run(["git", "cat-file", "-p", sha], capture_output=True).stdout.decode("utf-8", "replace")
        hits += [(f"history:{path}@{sha[:8]}", n, w) for _, n, w in scan_text(path, data, vals)]
    return len(names), n_blobs, hits, len(vals)


def main() -> int:
    if "--tree" in sys.argv:
        n_files, n_blobs, hits, n_vals = scan_tree()
        for path, n, what in hits:
            print(f"SECRET? {path}:{n}: {what} (the text is not printed)")
        print(f"secrets scan (tree): {n_files} file(s) that would ship, {n_blobs} history blob(s), "
              f"{n_vals} .env value(s) searched for, {len(hits)} hit(s)")
        return 1 if hits else 0
    names = staged_names()
    hits = scan()
    for path, n, what in hits:
        print(f"SECRET? {path}:{n}: {what} (the text is not printed)")
    print(f"secrets scan: {len(names)} staged file(s), {len(hits)} hit(s)")
    return 1 if hits else 0


if __name__ == "__main__":
    sys.exit(main())

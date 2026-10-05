#!/usr/bin/env python
"""Frames on stdin -> an H.264 MP4 (PyAV, libx264): each frame a 4-byte big-endian length and a JPEG.

    node replay/render_mp4.mjs sessions/<id>      (spawns this; run it with the lerobot venv's python)
"""
from __future__ import annotations

import io
import struct
import sys

import av
from PIL import Image


def main(out: str, fps: int) -> int:
    box = av.open(out, mode="w")
    st = box.add_stream("libx264", rate=fps)
    st.pix_fmt, st.options = "yuv420p", {"crf": "18", "preset": "veryfast"}
    n, src = 0, sys.stdin.buffer
    while True:
        head = src.read(4)
        if len(head) < 4:
            break
        img = Image.open(io.BytesIO(src.read(struct.unpack(">I", head)[0]))).convert("RGB")
        if n == 0:
            st.width, st.height = img.size
        for pkt in st.encode(av.VideoFrame.from_image(img)):
            box.mux(pkt)
        n += 1
    for pkt in st.encode():
        box.mux(pkt)
    box.close()
    print(f"encoded {n} frames at {fps} fps -> {out}", flush=True)
    return 0 if n else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1], int(sys.argv[2])))

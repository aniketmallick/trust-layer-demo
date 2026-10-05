"""The palm centre of one detected hand: the mean of the wrist and the five knuckle-row landmarks, in pixels.
Pure functions, no imports beyond the standard library."""
from __future__ import annotations

from typing import Sequence

N_LANDMARKS = 21
# MediaPipe hand landmarks: 0 wrist, 1 thumb CMC (the thumb's base), 5 / 9 / 13 / 17 the index / middle / ring / pinky MCP
PALM_LANDMARKS = (0, 1, 5, 9, 13, 17)


def landmarks_to_px(normalised: Sequence[Sequence[float]], width: int, height: int) -> list[tuple[float, float]]:
    """MediaPipe's normalised (x, y) in [0, 1] of the image -> pixels (u, v)."""
    return [(float(p[0]) * width, float(p[1]) * height) for p in normalised]


def palm_centre_px(landmarks_px: Sequence[Sequence[float]]) -> tuple[float, float]:
    """Mean of landmarks 0, 1, 5, 9, 13, 17 (u, v). The hand must carry all 21 landmarks."""
    if len(landmarks_px) != N_LANDMARKS:
        raise ValueError(f"a hand has {N_LANDMARKS} landmarks, got {len(landmarks_px)}")
    pts = [landmarks_px[i] for i in PALM_LANDMARKS]
    return (sum(float(p[0]) for p in pts) / len(pts), sum(float(p[1]) for p in pts) / len(pts))

"""MediaPipe Hand Landmarker (Tasks API, the only hand API in mediapipe 0.10.35) behind one small class.

IMAGE mode: every frame is detected on its own, with no tracking state carried from the frame before - a frame's
result depends on that frame only, so a saved still reads the same in a fixture as it did live.
Only this file imports mediapipe. It runs in the monitor's venv (.venv_monitor), never in the runner's.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from monitor.palm import landmarks_to_px

MIN_CONFIDENCE = 0.5          # MediaPipe's default for detection and presence; lower = more hands reported


@dataclass(frozen=True)
class RawHand:
    handedness: str            # "L" | "R": the model's label, as given (no rule uses it)
    conf: float                # the score of that label (the one per-hand score the Tasks API returns)
    landmarks_px: tuple        # 21 x (u, v), pixels of the frame


def mediapipe_version() -> str:
    import mediapipe as mp
    return str(mp.__version__)


class HandDetector:
    def __init__(self, model_path: Path, max_hands: int = 2, min_confidence: float = MIN_CONFIDENCE):
        import mediapipe as mp
        from mediapipe.tasks.python import BaseOptions, vision
        self._mp = mp
        opts = vision.HandLandmarkerOptions(base_options=BaseOptions(model_asset_path=str(model_path)),
                                            running_mode=vision.RunningMode.IMAGE, num_hands=int(max_hands),
                                            min_hand_detection_confidence=float(min_confidence),
                                            min_hand_presence_confidence=float(min_confidence))
        self._landmarker = vision.HandLandmarker.create_from_options(opts)
        self.max_hands = int(max_hands)

    def detect(self, bgr: np.ndarray) -> list[RawHand]:
        """One BGR frame (as cv2 reads it) -> the hands in it, in the model's order."""
        h, w = bgr.shape[:2]
        rgb = np.ascontiguousarray(bgr[:, :, ::-1])
        res = self._landmarker.detect(self._mp.Image(image_format=self._mp.ImageFormat.SRGB, data=rgb))
        hands = []
        for lms, cats in zip(res.hand_landmarks, res.handedness):
            label = "L" if str(cats[0].category_name).lower().startswith("l") else "R"
            px = landmarks_to_px([(lm.x, lm.y) for lm in lms], w, h)
            hands.append(RawHand(handedness=label, conf=float(cats[0].score), landmarks_px=tuple(px)))
        return hands

    def close(self) -> None:
        self._landmarker.close()

#!/usr/bin/env python3
"""
Build BioMotion offline keypoints from a video.

Implements the file format defined in OFFLINE_MODE_PLAN.md
(`biomotion-offline-keypoints-v1`):

  <output>.keypoints.json    metadata
  <output>.keypoints.bin     frame-major float32 [x, y, confidence] per point

OpenPose-25 keypoint ordering is the canonical layout consumed by the iOS app.
The default detector is MediaPipe Pose (33 landmarks) remapped into the OpenPose-25
slots; missing slots (LSmallToe, RSmallToe) are written as zeros so the binary
remains rectangular and the slot indices stay deterministic.

OpenCap's own OpenPose / mmpose detector pipeline can be plugged in later by
implementing the Detector interface. The keypoint file consumer on the iOS side
only cares about the OpenPose-25 layout and the binary stride, not the upstream
detector.

Example
-------
    python scripts/build_offline_keypoints.py \
        demodata/opencap-demodata/trail2/run2.mov \
        --output demodata/opencap-demodata/trail2/run2

Produces:
    demodata/opencap-demodata/trail2/run2.keypoints.json
    demodata/opencap-demodata/trail2/run2.keypoints.bin

Dependencies
------------
    pip install mediapipe opencv-python numpy
"""

from __future__ import annotations

import argparse
import json
import math
import os
import struct
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Sequence


# ---------------------------------------------------------------------------
# OpenPose-25 layout — must match BioMotion/Offline/OfflineKeypointTrack.swift
# ---------------------------------------------------------------------------

OPENPOSE_25 = [
    "Nose",       # 0
    "Neck",       # 1   derived from L/R shoulders
    "RShoulder",  # 2
    "RElbow",     # 3
    "RWrist",     # 4
    "LShoulder",  # 5
    "LElbow",     # 6
    "LWrist",     # 7
    "midHip",     # 8   derived from L/R hips
    "RHip",       # 9
    "RKnee",      # 10
    "RAnkle",     # 11
    "LHip",       # 12
    "LKnee",      # 13
    "LAnkle",     # 14
    "REye",       # 15
    "LEye",       # 16
    "REar",       # 17
    "LEar",       # 18
    "LBigToe",    # 19
    "LSmallToe",  # 20  not produced by MediaPipe — written as zero
    "LHeel",      # 21
    "RBigToe",    # 22
    "RSmallToe",  # 23  not produced by MediaPipe — written as zero
    "RHeel",      # 24
]

CHANNELS_PER_POINT = 3   # x, y, confidence
BYTES_PER_POINT = CHANNELS_PER_POINT * 4
FORMAT_TAG = "biomotion-offline-keypoints-v1"


# ---------------------------------------------------------------------------
# Detector interface
# ---------------------------------------------------------------------------


@dataclass
class Frame:
    """One detected frame in OpenPose-25 layout, image pixel space."""

    points: List[tuple]  # (x, y, confidence) tuples; len == len(OPENPOSE_25)


class Detector:
    """Subclass and override `detect()` to plug in another pose source."""

    name: str = "abstract"

    def open(self, video_path: Path) -> tuple[int, int, float, int]:
        """Return (width, height, fps, frame_count)."""
        raise NotImplementedError

    def frames(self) -> Iterable[Frame]:
        raise NotImplementedError

    def close(self) -> None:
        pass


class MediaPipeDetector(Detector):
    """MediaPipe Pose → OpenPose-25 mapping."""

    name = "opencap-openpose-25"

    # MediaPipe pose landmark indices we need.
    # Reference: https://developers.google.com/mediapipe/solutions/vision/pose_landmarker
    NOSE = 0
    LEFT_EYE = 2
    RIGHT_EYE = 5
    LEFT_EAR = 7
    RIGHT_EAR = 8
    LEFT_SHOULDER = 11
    RIGHT_SHOULDER = 12
    LEFT_ELBOW = 13
    RIGHT_ELBOW = 14
    LEFT_WRIST = 15
    RIGHT_WRIST = 16
    LEFT_HIP = 23
    RIGHT_HIP = 24
    LEFT_KNEE = 25
    RIGHT_KNEE = 26
    LEFT_ANKLE = 27
    RIGHT_ANKLE = 28
    LEFT_HEEL = 29
    RIGHT_HEEL = 30
    LEFT_FOOT_INDEX = 31   # mediapipe's "big toe" surrogate
    RIGHT_FOOT_INDEX = 32

    # mediapipe 0.10+ ships only the `mp.tasks.vision.PoseLandmarker` API on
    # Python 3.13/3.14 wheels — the legacy `mp.solutions.pose` namespace is no
    # longer present. We download the .task asset on first run and cache it
    # under ~/.cache/biomotion-keypoints/ so subsequent invocations are offline.
    MODEL_URLS = {
        0: "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_lite/float16/latest/pose_landmarker_lite.task",
        1: "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_full/float16/latest/pose_landmarker_full.task",
        2: "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_heavy/float16/latest/pose_landmarker_heavy.task",
    }

    def __init__(self, model_complexity: int = 1) -> None:
        if model_complexity not in self.MODEL_URLS:
            raise SystemExit(
                f"model_complexity must be 0, 1, or 2 (got {model_complexity})"
            )
        self.model_complexity = model_complexity
        self._cap = None
        self._landmarker = None
        self._mp = None
        self._width = 0
        self._height = 0
        self._fps = 0.0
        self._frame_count = 0
        self._frame_index = 0

    def open(self, video_path: Path) -> tuple[int, int, float, int]:
        try:
            import cv2  # noqa: F401
            import mediapipe as mp
            from mediapipe.tasks import python as mp_python
            from mediapipe.tasks.python import vision as mp_vision
        except ImportError as exc:  # pragma: no cover — diagnostic
            raise SystemExit(
                "MediaPipeDetector requires `mediapipe>=0.10` and `opencv-python`. "
                f"Install with `pip install mediapipe opencv-python`. ({exc})"
            )

        import cv2

        self._cv2 = cv2
        self._mp = mp
        cap = cv2.VideoCapture(str(video_path))
        # Respect the rotation tag baked into HEVC/MOV containers so frame
        # pixels match what AVPlayer renders on iOS.
        try:
            cap.set(cv2.CAP_PROP_ORIENTATION_AUTO, 1)
        except AttributeError:
            pass

        if not cap.isOpened():
            raise SystemExit(f"Could not open video: {video_path}")

        self._cap = cap
        self._width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self._height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self._fps = float(cap.get(cv2.CAP_PROP_FPS))
        self._frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

        model_path = self._ensure_model_cached(self.model_complexity)
        options = mp_vision.PoseLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=str(model_path)),
            running_mode=mp_vision.RunningMode.VIDEO,
            num_poses=1,
            min_pose_detection_confidence=0.5,
            min_pose_presence_confidence=0.5,
            min_tracking_confidence=0.5,
            output_segmentation_masks=False,
        )
        self._landmarker = mp_vision.PoseLandmarker.create_from_options(options)
        self._frame_index = 0

        return self._width, self._height, self._fps, self._frame_count

    @staticmethod
    def _ensure_model_cached(complexity: int) -> Path:
        cache_dir = Path.home() / ".cache" / "biomotion-keypoints"
        cache_dir.mkdir(parents=True, exist_ok=True)
        url = MediaPipeDetector.MODEL_URLS[complexity]
        target = cache_dir / Path(url).name
        if target.exists() and target.stat().st_size > 0:
            return target
        print(f"Downloading {url} -> {target}", file=sys.stderr)
        import urllib.request
        with urllib.request.urlopen(url) as resp, target.open("wb") as fp:
            fp.write(resp.read())
        return target

    def frames(self) -> Iterable[Frame]:
        assert self._cap is not None and self._landmarker is not None

        cv2 = self._cv2
        mp = self._mp
        zeros = (0.0, 0.0, 0.0)
        # Use synthetic timestamps spaced by 1/fps so detect_for_video sees a
        # monotonic timeline; container PTS is unreliable on rotated MOV.
        timestamp_step_ms = max(1, int(round(1000.0 / max(self._fps, 1.0))))

        while True:
            ok, bgr = self._cap.read()
            if not ok:
                break
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            timestamp_ms = self._frame_index * timestamp_step_ms
            self._frame_index += 1
            result = self._landmarker.detect_for_video(mp_image, timestamp_ms)

            if not result.pose_landmarks:
                yield Frame(points=[zeros] * len(OPENPOSE_25))
                continue

            lm = result.pose_landmarks[0]
            w, h = self._width, self._height

            def pt(i: int) -> tuple:
                p = lm[i]
                return (float(p.x) * w, float(p.y) * h, float(p.visibility))

            def midpoint(i: int, j: int) -> tuple:
                a, b = pt(i), pt(j)
                return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, min(a[2], b[2]))

            mp_to_openpose = [
                pt(self.NOSE),                                                  # 0  Nose
                midpoint(self.LEFT_SHOULDER, self.RIGHT_SHOULDER),              # 1  Neck
                pt(self.RIGHT_SHOULDER),                                        # 2  RShoulder
                pt(self.RIGHT_ELBOW),                                           # 3  RElbow
                pt(self.RIGHT_WRIST),                                           # 4  RWrist
                pt(self.LEFT_SHOULDER),                                         # 5  LShoulder
                pt(self.LEFT_ELBOW),                                            # 6  LElbow
                pt(self.LEFT_WRIST),                                            # 7  LWrist
                midpoint(self.LEFT_HIP, self.RIGHT_HIP),                        # 8  midHip
                pt(self.RIGHT_HIP),                                             # 9  RHip
                pt(self.RIGHT_KNEE),                                            # 10 RKnee
                pt(self.RIGHT_ANKLE),                                           # 11 RAnkle
                pt(self.LEFT_HIP),                                              # 12 LHip
                pt(self.LEFT_KNEE),                                             # 13 LKnee
                pt(self.LEFT_ANKLE),                                            # 14 LAnkle
                pt(self.RIGHT_EYE),                                             # 15 REye
                pt(self.LEFT_EYE),                                              # 16 LEye
                pt(self.RIGHT_EAR),                                             # 17 REar
                pt(self.LEFT_EAR),                                              # 18 LEar
                pt(self.LEFT_FOOT_INDEX),                                       # 19 LBigToe
                zeros,                                                          # 20 LSmallToe (n/a)
                pt(self.LEFT_HEEL),                                             # 21 LHeel
                pt(self.RIGHT_FOOT_INDEX),                                      # 22 RBigToe
                zeros,                                                          # 23 RSmallToe (n/a)
                pt(self.RIGHT_HEEL),                                            # 24 RHeel
            ]

            yield Frame(points=mp_to_openpose)

    def close(self) -> None:
        if self._landmarker is not None:
            self._landmarker.close()
            self._landmarker = None
        if self._cap is not None:
            self._cap.release()
            self._cap = None


# ---------------------------------------------------------------------------
# Writer
# ---------------------------------------------------------------------------


def write_outputs(
    detector: Detector,
    video_path: Path,
    output_stem: Path,
) -> tuple[Path, Path, int]:
    """Run `detector` on `video_path` and write the keypoints pair.

    `output_stem` is the path prefix (the suffixes `.keypoints.json` and
    `.keypoints.bin` are appended).
    """
    width, height, fps, declared_frame_count = detector.open(video_path)

    json_path = output_stem.with_suffix("")
    json_path = Path(str(json_path) + ".keypoints.json")
    bin_path = output_stem.with_suffix("")
    bin_path = Path(str(bin_path) + ".keypoints.bin")

    json_path.parent.mkdir(parents=True, exist_ok=True)

    written_frames = 0
    bin_buf = bytearray()
    point_pack = struct.Struct("<fff").pack  # little-endian float32 x3

    try:
        for frame in detector.frames():
            if len(frame.points) != len(OPENPOSE_25):
                raise SystemExit(
                    f"Detector returned {len(frame.points)} points, "
                    f"expected {len(OPENPOSE_25)} for OpenPose-25."
                )
            for x, y, c in frame.points:
                if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(c)):
                    x = y = c = 0.0
                bin_buf += point_pack(float(x), float(y), float(c))
            written_frames += 1
    finally:
        detector.close()

    bin_path.write_bytes(bytes(bin_buf))

    metadata = {
        "format": FORMAT_TAG,
        "video": {
            "filename": video_path.name,
            "width": width,
            "height": height,
            "fps": fps,
            # Prefer the count we actually wrote; container metadata can lie.
            "frame_count": written_frames,
        },
        "keypoint_set": {
            "name": detector.name,
            "points": OPENPOSE_25,
        },
        "storage": {
            "binary_filename": bin_path.name,
            "scalar_type": "float32",
            "channels_per_point": CHANNELS_PER_POINT,
            "channel_order": ["x", "y", "confidence"],
            "layout": "frame-major",
        },
    }

    if declared_frame_count and declared_frame_count != written_frames:
        metadata["video"]["declared_frame_count"] = declared_frame_count

    with json_path.open("w") as fp:
        json.dump(metadata, fp, indent=2)

    return json_path, bin_path, written_frames


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate BioMotion offline 2D pose keypoint files.",
    )
    parser.add_argument(
        "video",
        type=Path,
        help="Path to the source video (e.g. demodata/.../run2.mov).",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help=(
            "Output stem (suffixes .keypoints.json/.keypoints.bin are appended). "
            "Defaults to the video's path with the extension stripped."
        ),
    )
    parser.add_argument(
        "--detector",
        choices=["mediapipe"],
        default="mediapipe",
        help="Detector backend (default: mediapipe).",
    )
    parser.add_argument(
        "--model-complexity",
        type=int,
        default=1,
        help="MediaPipe model_complexity (0=lite, 1=full, 2=heavy).",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)

    if not args.video.exists():
        print(f"error: video not found: {args.video}", file=sys.stderr)
        return 2

    output_stem = args.output if args.output is not None else args.video.with_suffix("")

    if args.detector == "mediapipe":
        detector: Detector = MediaPipeDetector(model_complexity=args.model_complexity)
    else:
        raise SystemExit(f"unsupported detector: {args.detector}")

    json_path, bin_path, frame_count = write_outputs(detector, args.video, output_stem)

    bin_size_kb = os.path.getsize(bin_path) / 1024.0
    print(f"wrote {json_path}")
    print(f"wrote {bin_path} ({bin_size_kb:.1f} KB, {frame_count} frames)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

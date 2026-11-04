import json
from pathlib import Path

import numpy as np


POSES_JSON = Path("poses.json")
OUTPUT_DIR = Path("point_robot")
EXPECTED_POSE_COUNT = 19


def main():
    with POSES_JSON.open("r", encoding="utf-8") as f:
        data = json.load(f)

    poses = data.get("poses")
    if not isinstance(poses, list):
        raise ValueError("poses.json must contain a top-level 'poses' list")

    if len(poses) != EXPECTED_POSE_COUNT:
        raise ValueError(f"Expected {EXPECTED_POSE_COUNT} poses, got {len(poses)}")

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    for index, pose in enumerate(poses, start=1):
        pose_array = np.asarray(pose, dtype=np.float64)
        np.save(OUTPUT_DIR / f"result{index}.npy", pose_array)

    print(f"Saved {len(poses)} npy files to {OUTPUT_DIR}")


if __name__ == "__main__":
    main()

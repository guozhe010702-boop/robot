"""Convert the adjacent output.txt six-component mm/rad samples to NPY."""
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent


def save_poses(txt_path=HERE / "output.txt", save_dir=HERE / "result"):
    poses = []
    with Path(txt_path).open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                pose = np.array([float(v) for v in line.split()], dtype=float)
            except ValueError as exc:
                raise ValueError(f"第 {line_number} 行包含非数值") from exc
            if pose.shape != (6,) or not np.isfinite(pose).all():
                raise ValueError(f"第 {line_number} 行必须为六个有限数值：x y z rx ry rz（mm/rad）")
            poses.append(pose)
    # Validate every row before writing any converted sample.
    save_dir = Path(save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)
    for index, pose in enumerate(poses, 1):
        path = save_dir / f"result{index}.npy"
        np.save(path, pose)
        print(f"Saved: {path}")
    return len(poses)


if __name__ == "__main__":
    save_poses()

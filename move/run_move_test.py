"""Manual test entry for move_to_pose.py, defaulting to 192.168.1.200:8055.

No arguments: query current state/pose. --pose: read-only motion preflight.
All execution requirements and interactive confirmation remain in move_to_pose.
Importing this file never connects to the robot.
python .\test\run_move_test.py --pose 100 300 500 3 0.1 1 
--expected-version V3.14.2 --expected-tool 0 --confirm-base-tcp-rad --execute
"""
from __future__ import annotations

from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import move_to_pose as motion

ROBOT_HOST = "192.168.1.200"
ROBOT_PORT = 8055


def main(argv=None):
    arguments = list(sys.argv[1:] if argv is None else argv)
    # Later explicit --host/--port options override these defaults in argparse.
    forwarded = ["--host", ROBOT_HOST, "--port", str(ROBOT_PORT), *arguments]
    if not any(arg in ("--pose", "--show-current", "--offline", "--execute")
               or arg.startswith("--pose=") for arg in arguments):
        forwarded.append("--show-current")
    return motion.main(forwarded)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("测试中断不代表机械臂已停止，请通过示教器确认状态。", file=sys.stderr)
        raise SystemExit(130)
    except (motion.ECSDKError, OSError, ValueError, RuntimeError) as exc:
        print(f"测试未完成：{exc}", file=sys.stderr)
        raise SystemExit(1)

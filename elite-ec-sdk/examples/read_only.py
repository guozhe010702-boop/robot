"""Read-only connection check; this file does not change robot state."""

from elite_ec_sdk import ECClient, ECSDKError


def main() -> None:
    try:
        with ECClient("192.168.1.200", timeout=3) as robot:
            print("version:", robot.get_soft_version())
            print("mode:", robot.get_robot_mode())
            print("state:", robot.get_robot_state())
            print("estop:", robot.get_estop_status())
    except (ECSDKError, OSError, TimeoutError) as exc:
        print(f"Read-only check failed: {exc}")


if __name__ == "__main__":
    main()

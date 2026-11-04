"""Python client for the ELITE EC/EA controller's 8055 Socket API.

Importing this package never connects to or changes a robot.
"""

from .client import (
    ECClient,
    ECSDKError,
    DisconnectedError,
    ProtocolError,
    RPCError,
    UnsafeCallError,
)

__all__ = [
    "ECClient", "ECSDKError", "DisconnectedError", "ProtocolError",
    "RPCError", "UnsafeCallError",
]

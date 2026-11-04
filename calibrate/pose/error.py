class FuelingError(Exception):
    """Base class for exceptions in this module."""
    pass

class RobotControlError(FuelingError):
    def __init__(self, message="Robot control error occurred"):
        super().__init__(message)
        self.message = message

class RobotRemoteError(FuelingError):
    def __init__(self, exception):
        super().__init__(f"Remote robot error: {exception}")
        self.exception = exception

class RecvTimeoutError(TimeoutError):
    """Socket 接收数据时超时"""
    def __init__(self, timeout: float, peer: str = "") -> None:
        super().__init__(f"recv timeout after {timeout:.2f}s from {peer or 'peer'}")
        self.timeout = timeout
        self.peer = peer

class PeerClosedError(ConnectionError):
    """对端已关闭连接（recv 返回空字节）"""
    def __init__(self, peer: str = "") -> None:
        super().__init__(f"peer {peer or 'unknown'} closed the connection")
        self.peer = peer

class SocketConnectError(ConnectionError):
    """Socket 连接阶段失败（超时、拒绝、不可达等）"""
    def __init__(self, host: str, port: int, timeout: float, reason: str = "") -> None:
        msg = f"connect to {host}:{port} timeout={timeout:.2f}s failed"
        if reason:
            msg += f" ({reason})"
        super().__init__(msg)
        self.host = host
        self.port = port
        self.timeout = timeout
        self.reason = reason

class DataReceiveError(OSError):
    """底层接收数据失败（超时、对端关闭、校验错误等）"""
    def __init__(
        self,
        expected: int,
        actual: int,
        peer: str = "",
        timeout: float | None = None,
        info: str | None = None
    ) -> None:
        msg = f"data receive failed: expected {expected} bytes, got {actual}, info: {info or 'N/A'}"
        if peer:
            msg += f" from {peer}"
        if timeout is not None:
            msg += f" (timeout {timeout:.2f}s)"
        super().__init__(msg)
        self.expected = expected
        self.actual   = actual
        self.peer     = peer
        self.timeout  = timeout

class OrbbecCameraError(Exception):
    """Base exception for Orbbec camera operations."""
    pass

class IRImageDecodeError(OrbbecCameraError):
    """Raised when IR image decoding fails."""

    def __init__(self, format_type: str, message: str = "Failed to decode IR image data"):
        self.format_type = format_type
        self.message = f"{message} for format: {format_type}"
        super().__init__(self.message)
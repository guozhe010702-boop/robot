import socket
import anyio
import pandas
import struct
from loguru import logger
from error import RobotRemoteError, SocketConnectError, DataReceiveError
from typing import Optional


def unpack_header(buf: bytes) -> tuple[int, int]:
    (size, type) =  struct.unpack_from('>iB', buf)
    return size, type

def type_to_pack(vartype : str):
    if vartype == 'int' or vartype == 'int32_t':
        return 'i'
    elif vartype == 'int8_t' :
        return 'b'
    elif vartype == 'uint8_t':
        return 'B'
    elif vartype == 'uint64_t':
        return'Q'
    elif vartype == 'bool':
        return '?'
    elif vartype == 'double':
        return 'd'
    elif vartype == 'uint32_t':
        return 'I'
    elif vartype == 'float':
        return 'f'
    elif vartype == 'uint32_t':
        return 'I'
    else:
        raise ValueError("Unknown Type")

def get_config(file: str, sheet: str):
    config_fmt = '>'
    config_name = []
    excel = pandas.read_excel(file, sheet_name=sheet)
    is_foreach = False
    temp_fmt = ''
    temp_name = []
    elite_internel_count = 0
    for i in range(len(excel['type'])):
        if type(excel['name'][i]) == float:
            pass
        if type(excel['type'][i]) == str:
            excel['type'][i].replace(" ", "")
        if excel['type'][i] == 'bytes':
            config_fmt += 'B' * excel['bytes'][i]
            for j in range(excel['bytes'][i]):
                config_name.append(excel['name'][i] + '_' +str(j) + '_' + str(elite_internel_count))
                elite_internel_count += 1
        elif excel['type'][i] == 'foreach':
            is_foreach = True
        elif excel['type'][i] == 'end' and is_foreach == True:
            config_fmt += (temp_fmt * 6)
            for j in range(6):
                for k in temp_name:
                    config_name.append(k + str(j))
            temp_fmt = ''
            temp_name = []
            is_foreach = False
        else:
            if is_foreach:
                temp_name.append(excel['name'][i])
                temp_fmt += type_to_pack(excel['type'][i])
            else:
                config_name.append(excel['name'][i])
                config_fmt += type_to_pack(excel['type'][i])
    return (config_fmt, config_name)

ROBOT_STATE_TYPE = 16
ROBOT_EXCEPTION = 20

def unpack_message(buf: bytes, config: tuple) -> dict:
    config_fmt, config_name = config
    unpack = struct.unpack_from(config_fmt, buf)
    return dict(zip(config_name, unpack))

# Robot exception message type
ROBOT_MESSAGE_RUNTIME_EXCEPTION = 10
ROBOT_MESSAGE_EXCEPTION = 6

# Robot exception data type
ROBOT_EXCEPTION_DATA_TYPE_NONE = 0
ROBOT_EXCEPTION_DATA_TYPE_UNSIGNED = 1
ROBOT_EXCEPTION_DATA_TYPE_SIGNED = 2
ROBOT_EXCEPTION_DATA_TYPE_FLOAT = 3
ROBOT_EXCEPTION_DATA_TYPE_HEX = 4
ROBOT_EXCEPTION_DATA_TYPE_STRING = 5
ROBOT_EXCEPTION_DATA_TYPE_JOINT = 6

def unpack_exception(buffer: bytes):
    fmt = '>iBQBB'
    (pack_len, _, timestamp, source, msg_type) = struct.unpack_from(fmt, buffer)
    offset = struct.calcsize(fmt)

    exception = {
        'timestamp': timestamp,
        'source': source,
        'msg_type': msg_type,
    }

    if msg_type == ROBOT_MESSAGE_RUNTIME_EXCEPTION:
        fmt = '>ii'
        script_line, script_column = struct.unpack_from(fmt, buffer, offset)
        offset += struct.calcsize(fmt)

        description = buffer[offset:pack_len].decode('utf-8')
        exception.update({
            'script_line': script_line,
            'script_column': script_column,
            'description': description
        })
        return exception

    elif msg_type == ROBOT_MESSAGE_EXCEPTION:
        fmt = '>iiii'
        (_, _, _, data_type) = struct.unpack_from(fmt, buffer, offset)
        if data_type == ROBOT_EXCEPTION_DATA_TYPE_NONE:
            fmt = '>iiiiI'
            (code, subcode, level, _, data) = struct.unpack_from(fmt, buffer, offset)
        elif data_type == ROBOT_EXCEPTION_DATA_TYPE_UNSIGNED:
            fmt = '>iiiiI'
            (code, subcode, level, _, data) = struct.unpack_from(fmt, buffer, offset)
        elif data_type == ROBOT_EXCEPTION_DATA_TYPE_SIGNED:
            fmt = '>iiiii'
            (code, subcode, level, _, data) = struct.unpack_from(fmt, buffer, offset)
        elif data_type == ROBOT_EXCEPTION_DATA_TYPE_FLOAT:
            fmt = '>iiiif'
            (code, subcode, level, _, data) = struct.unpack_from(fmt, buffer, offset)
        elif data_type == ROBOT_EXCEPTION_DATA_TYPE_HEX:
            fmt = '>iiiiI'
            (code, subcode, level, _, data) = struct.unpack_from(fmt, buffer, offset)
        elif data_type == ROBOT_EXCEPTION_DATA_TYPE_STRING:
            offset += struct.calcsize(fmt)
            data = buffer[offset : pack_len]
        elif data_type == ROBOT_EXCEPTION_DATA_TYPE_JOINT:
            fmt = 'iiiiI'
            (code, subcode, level, _, data) = struct.unpack_from(fmt, buffer, offset)
        exception.update({
            'code': code,
            'subcode': subcode,
            'level': level,
            'data': data,
        })
        return exception


DEFAULT_TIMEOUT = 10.0

class RobotClient:
    def __init__(self, addr: str, port: int, excel_sheet: str, timeout: float = DEFAULT_TIMEOUT) -> None:
        excel, sheet = excel_sheet.split(':')
        self.data_config = get_config(excel, sheet)
        self.timeout = timeout
        self.addr = addr
        self.port = port

        self.connect()

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.sock.settimeout(self.timeout)
            self.sock.connect((self.addr, self.port))
        except (socket.timeout, socket.error):
            self.sock = None
            raise SocketConnectError(self.addr, self.port, self.timeout)

    def disconnect(self):
        if not self.sock:
            raise SocketConnectError(self.addr, self.port, self.timeout)
        self.sock.close()
        self.sock = None

    def send(self, data: bytes | str):
        if not self.sock:
            raise SocketConnectError(self.addr, self.port, self.timeout)
        if isinstance(data, str):
            data = data.encode('utf-8')
        return self.sock.sendall(data)

    def recv(self, bufsize: int = 4096, timeout: Optional[float] = None):
        if timeout is None:
            timeout = self.timeout
        if not self.sock:
            raise SocketConnectError(self.addr, self.port, self.timeout)
        self.sock.settimeout(timeout)
        try:
            data = self.sock.recv(bufsize)
            if not data:  # 对端关闭连接
                return None
            return data
        except socket.timeout:
            # 超时没有收到数据
            logger.error("no data received within timeout")
            return None
        except Exception as e:
            raise DataReceiveError(bufsize, len(data), peer=f"{self.addr}:{self.port}", timeout=timeout, info=str(e))

    def recv_data(self, timeout: Optional[float] = None):
        if timeout is None:
            timeout = self.timeout
        buf = self.recv(4096, timeout=timeout)
        if buf is None:
            return None
        while(len(buf) > 5):
            size, type = unpack_header(buf)
            if size <= 0 or size > 10**6:
                # logger.error(f"Invalid header size={size}, buf[:10]={buf[:10].hex()}")
                # 丢弃一个字节重新同步包头
                buf = buf[1:]
                continue
            elif size <= len(buf):
                if type not in (ROBOT_STATE_TYPE, ROBOT_EXCEPTION):
                    buf = buf[size :]
                    continue
                # assert type in (ROBOT_STATE_TYPE, ROBOT_EXCEPTION), f"unsupported type {type}"
                # buf = self.recv(size, timeout=timeout)
                if type == ROBOT_STATE_TYPE:
                    data = unpack_message(buf, self.data_config)
                    return data

                elif type == ROBOT_EXCEPTION:
                    exception = unpack_exception(buf)
                    raise RobotRemoteError(exception)
            else:
                break
        return None

class AsyncRobotConnection:
    def __init__(self, addr: str, port: int, timeout: float = DEFAULT_TIMEOUT) -> None:
        self.stream = None
        self.addr = addr
        self.port = port
        self.timeout = timeout

    async def connect(self):
        try:
            self.stream = await anyio.connect_tcp(self.addr, self.port)
        except Exception:
            self.stream = None
            raise SocketConnectError(self.addr, self.port, self.timeout)

    async def disconnect(self):
        if not self.stream:
            raise SocketConnectError(self.addr, self.port, self.timeout)
        await self.stream.aclose()
        self.stream = None

    async def send(self, data: bytes | str, sleep_interval: float = 0.1):
        if not self.stream:
            raise SocketConnectError(self.addr, self.port, self.timeout)
        if isinstance(data, str):
            data = data.encode('utf-8')
        await self.stream.send(data)
        if sleep_interval:
            await anyio.sleep(sleep_interval)

    async def recv(self, bufsize: int = 4096, timeout: Optional[float] = None):
        if timeout is None:
            timeout = self.timeout
        if not self.stream:
            raise SocketConnectError(self.addr, self.port, self.timeout)
        with anyio.move_on_after(timeout) as scope:
            return await self.stream.receive(bufsize)
        if scope.cancel_called:
            logger.error("no data received within timeout")
            return None

class AsyncRobotMessageConnection(AsyncRobotConnection):
    def __init__(self, addr: str, port: int, excel_sheet: str, timeout: float = DEFAULT_TIMEOUT) -> None:
        super().__init__(addr, port, timeout)
        excel, sheet = excel_sheet.split(':')
        self.data_config = get_config(excel, sheet)

    async def recv_data(self, timeout: Optional[float] = None):
        if timeout is None:
            timeout = self.timeout
        buf = await self.recv(4096, timeout=timeout)
        if buf is None:
            return None
        while(len(buf) > 5):
            size, type = unpack_header(buf)
            if size <= 0 or size > 10**6:
                # logger.error(f"Invalid header size={size}, buf[:10]={buf[:10].hex()}")
                # 丢弃一个字节重新同步包头
                buf = buf[1:]
                continue
            elif size <= len(buf):
                if type not in (ROBOT_STATE_TYPE, ROBOT_EXCEPTION):
                    buf = buf[size :]
                    continue
                # assert type in (ROBOT_STATE_TYPE, ROBOT_EXCEPTION), f"unsupported type {type}"
                # buf = self.recv(size, timeout=timeout)
                if type == ROBOT_STATE_TYPE:
                    data = unpack_message(buf, self.data_config)
                    return data

                elif type == ROBOT_EXCEPTION:
                    exception = unpack_exception(buf)
                    raise RobotRemoteError(exception)
            else:
                break
        return None

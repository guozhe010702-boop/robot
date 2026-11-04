import json
import os
import sys
import socket
import unittest
from unittest.mock import patch
# 获取当前文件的绝对路径
current_dir = os.path.dirname(os.path.abspath(__file__))
# 获取上一级目录的绝对路径
parent_dir = os.path.dirname(current_dir)

# 将上一级目录添加到系统路径中
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)


from elite_ec_sdk import ECClient, ProtocolError, RPCError, UnsafeCallError


class FakeSocket:
    def __init__(self, reply=None, *, split=False):
        self.reply = reply or (lambda req: {"jsonrpc": "2.0", "id": req["id"], "result": True})
        self.split = split
        self.sent = []
        self.chunks = []
        self.closed = False

    def settimeout(self, value):
        self.timeout = value

    def sendall(self, data):
        assert data.endswith(b"\n")
        request = json.loads(data)
        self.sent.append(request)
        response = self.reply(request)
        wire = json.dumps(response).encode() + b"\n"
        self.chunks.extend([wire[:5], wire[5:]] if self.split else [wire])

    def recv(self, _size):
        return self.chunks.pop(0) if self.chunks else b""

    def close(self):
        self.closed = True


class ECClientTests(unittest.TestCase):
    def client(self, fake, **kwargs):
        patcher = patch.object(socket, "create_connection", return_value=fake)
        self.addCleanup(patcher.stop)
        patcher.start()
        client = ECClient("192.168.1.200", **kwargs).connect()
        self.addCleanup(client.close)
        return client

    def test_read_only_and_split_reply(self):
        fake = FakeSocket(lambda req: {"jsonrpc": "2.0", "id": req["id"],
                                       "result": "V3.14.2"}, split=True)
        client = self.client(fake)
        self.assertEqual(client.get_soft_version(), "V3.14.2")
        self.assertEqual(fake.sent[0]["method"], "getSoftVersion")
        self.assertEqual(fake.sent[0]["jsonrpc"], "2.0")
        self.assertEqual(fake.sent[0]["params"], [])

    def test_explicit_params_preserved(self):
        fake = FakeSocket(lambda req: {"jsonrpc": "2.0", "id": req["id"],
                                       "result": [1, 2, 3, 0, 0, 0]})
        client = self.client(fake)
        self.assertEqual(client.get_base_flange_pose(unit_type=1), [1, 2, 3, 0, 0, 0])
        self.assertEqual(fake.sent[0]["params"], {"unit_type": 1})

    def test_unknown_and_control_method_are_blocked_by_default(self):
        fake = FakeSocket()
        client = self.client(fake)
        for method in ("set_servo_status", "moveByJoint", "getUnreviewedStatus"):
            with self.assertRaises(UnsafeCallError):
                client.call(method, control=True)
        self.assertEqual(fake.sent, [])

    def test_control_needs_two_opt_ins(self):
        fake = FakeSocket()
        client = self.client(fake, allow_control=True)
        with self.assertRaises(UnsafeCallError):
            client.call("set_servo_status", {"status": 1})
        self.assertTrue(client.call("set_servo_status", {"status": 1}, control=True))
        self.assertEqual(fake.sent[0]["method"], "set_servo_status")

    def test_rpc_error_exposes_code_and_closes(self):
        fake = FakeSocket(lambda req: {"jsonrpc": "2.0", "id": req["id"],
                                       "error": {"code": -32601, "message": "Method not found"}})
        client = self.client(fake)
        with self.assertRaises(RPCError) as caught:
            client.get_base_flange_pose()
        self.assertEqual(caught.exception.code, -32601)
        self.assertFalse(client.connected)

    def test_reply_id_mismatch_closes(self):
        fake = FakeSocket(lambda req: {"jsonrpc": "2.0", "id": req["id"] + 1,
                                       "result": 2})
        client = self.client(fake)
        with self.assertRaises(ProtocolError):
            client.get_robot_mode()
        self.assertFalse(client.connected)

    def test_string_encoded_json_result(self):
        fake = FakeSocket(lambda req: {"jsonrpc": "2.0", "id": req["id"],
                                       "result": "false"})
        client = self.client(fake)
        self.assertFalse(client.get_servo_status())

    def test_bad_pose_is_rejected_before_send(self):
        fake = FakeSocket()
        client = self.client(fake)
        with self.assertRaises(ValueError):
            client.pose_to_matrix([0, 0, 0, float("nan"), 0, 0])
        self.assertEqual(fake.sent, [])


if __name__ == "__main__":
    unittest.main()

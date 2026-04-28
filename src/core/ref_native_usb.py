"""Shared REF native-USB fibre transport."""

from __future__ import annotations

import json
import sys
import threading
from pathlib import Path
from typing import Callable, TypeVar


CORE_DIR = Path(__file__).resolve().parent
SRC_DIR = CORE_DIR.parent
PROJECT_DIR = SRC_DIR.parent
CLI_TOOL_DIR = PROJECT_DIR / "third_party" / "CLI-Tool"


if str(CLI_TOOL_DIR) not in sys.path:
    sys.path.insert(0, str(CLI_TOOL_DIR))


RemoteT = TypeVar("RemoteT")


class RefNativeUsbLink:
    """Keep a single shared native USB fibre session to the REF controller."""

    VID = 0x1209
    PID = 0x0D32

    def __init__(self, channel_name: str = "dummy-controller"):
        self.channel_name = channel_name
        self._lock = threading.Lock()
        self._device = None
        self._transport = None
        self._channel = None
        self._remote = None
        self._last_error = ""

    def call(self, callback: Callable[[RemoteT], RemoteT]):
        with self._lock:
            for _ in range(2):
                try:
                    remote = self._ensure_remote_locked()
                    result = callback(remote)
                    self._last_error = ""
                    return result
                except Exception as exc:
                    self._last_error = str(exc)
                    self._invalidate_locked()

            raise RuntimeError(self._last_error or "REF native USB 调用失败")

    def is_connected(self) -> bool:
        with self._lock:
            return self._remote is not None

    def get_last_error(self) -> str:
        with self._lock:
            return self._last_error

    def close(self):
        with self._lock:
            self._invalidate_locked()

    def _ensure_remote_locked(self):
        if self._remote is not None:
            return self._remote

        import usb.core
        import fibre.protocol
        import fibre.remote_object
        from fibre.utils import Logger
        from fibre.usbbulk_transport import USBBulkTransport

        dev = usb.core.find(idVendor=self.VID, idProduct=self.PID)
        if dev is None:
            raise RuntimeError("未找到 REF 原生 USB 接口")

        logger = Logger(verbose=False)
        transport = USBBulkTransport(dev, logger)
        transport.init()
        channel = fibre.protocol.Channel(
            self.channel_name, transport, transport, None, logger
        )

        json_bytes = channel.remote_endpoint_read_buffer(0)
        channel._interface_definition_crc = fibre.protocol.calc_crc16(
            fibre.protocol.PROTOCOL_VERSION, json_bytes
        )
        root = {
            "name": "fibre_node",
            "members": json.loads(json_bytes.decode("ascii")),
        }
        remote = fibre.remote_object.RemoteObject(root, None, channel, logger)

        self._device = dev
        self._transport = transport
        self._channel = channel
        self._remote = remote
        return remote

    def _invalidate_locked(self):
        if self._transport is not None:
            try:
                self._transport.deinit()
            except Exception:
                pass

        self._device = None
        self._transport = None
        self._channel = None
        self._remote = None

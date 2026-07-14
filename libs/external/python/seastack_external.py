"""
SEA-Stack external force module client (Python).

Speaks the v1 length-prefixed JSON protocol over TCP to 127.0.0.1.
See docs/extending/EXTERNAL_FORCE_MODULES.md.
"""

from __future__ import annotations

import argparse
import json
import socket
import struct
import sys
from typing import Any, Dict, List, Optional


PROTOCOL_VERSION = 1


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("socket closed while receiving")
        buf.extend(chunk)
    return bytes(buf)


def recv_message(sock: socket.socket) -> dict:
    hdr = _recv_exact(sock, 4)
    (length,) = struct.unpack(">I", hdr)
    payload = _recv_exact(sock, length) if length else b""
    return json.loads(payload.decode("utf-8"))


def send_message(sock: socket.socket, obj: dict) -> None:
    payload = json.dumps(obj, separators=(",", ":")).encode("utf-8")
    sock.sendall(struct.pack(">I", len(payload)) + payload)


class ExternalForceModule:
    """Subclass and implement initialize / evaluate / shutdown."""

    def initialize(self, cfg: Dict[str, Any]) -> Dict[str, Any]:
        return {"name": self.__class__.__name__, "version": "1.0", "n_states": 0}

    def evaluate(self, t: float, dt: float, inputs: List[float]) -> List[float]:
        raise NotImplementedError

    def reset(self) -> None:
        pass

    def commit(self) -> None:
        pass

    def rollback(self) -> None:
        pass

    def shutdown(self) -> None:
        pass

    @classmethod
    def run(cls, instance: Optional["ExternalForceModule"] = None) -> None:
        module = instance if instance is not None else cls()
        parser = argparse.ArgumentParser(description="SEA-Stack external force module")
        parser.add_argument("--seastack-port", type=int, required=True)
        args = parser.parse_args()

        sock = socket.create_connection(("127.0.0.1", args.seastack_port), timeout=30.0)
        sock.settimeout(60.0)
        try:
            while True:
                msg = recv_message(sock)
                op = msg.get("op")
                if op == "initialize":
                    proto = int(msg.get("protocol", 0))
                    if proto and proto != PROTOCOL_VERSION:
                        send_message(
                            sock,
                            {
                                "status": "error",
                                "message": f"unsupported protocol {proto}",
                            },
                        )
                        break
                    cfg = msg.get("config") or {}
                    if isinstance(cfg, str):
                        cfg = json.loads(cfg) if cfg else {}
                    meta = module.initialize(cfg) or {}
                    send_message(
                        sock,
                        {
                            "status": "ok",
                            "protocol": PROTOCOL_VERSION,
                            "name": meta.get("name", module.__class__.__name__),
                            "version": meta.get("version", "1.0"),
                            "n_states": int(meta.get("n_states", 0)),
                        },
                    )
                elif op == "evaluate":
                    t = float(msg.get("t", 0.0))
                    dt = float(msg.get("dt", 0.0))
                    inputs = [float(x) for x in msg.get("in", [])]
                    try:
                        outputs = module.evaluate(t, dt, inputs)
                        send_message(
                            sock,
                            {"status": "ok", "out": [float(x) for x in outputs]},
                        )
                    except Exception as exc:  # noqa: BLE001 — report to host
                        send_message(
                            sock, {"status": "error", "message": str(exc)}
                        )
                elif op == "reset":
                    module.reset()
                    send_message(sock, {"status": "ok"})
                elif op == "commit":
                    module.commit()
                    send_message(sock, {"status": "ok"})
                elif op == "rollback":
                    module.rollback()
                    send_message(sock, {"status": "ok"})
                elif op == "shutdown":
                    module.shutdown()
                    send_message(sock, {"status": "ok"})
                    break
                else:
                    send_message(
                        sock,
                        {"status": "error", "message": f"unknown op: {op}"},
                    )
        finally:
            sock.close()


if __name__ == "__main__":
    sys.stderr.write(
        "seastack_external.py is the IPC helper only.\n"
        "Run a demo module instead, e.g.\n"
        "  demos/rm3/external_pto/linear_damper_pto.py\n"
        "  demos/rm3/external_pto_adaptive/adaptive_damping_pto.py\n"
        "  demos/rm3/external_pto_hydraulic/hydraulic_accumulator_pto.py\n"
    )
    sys.exit(2)

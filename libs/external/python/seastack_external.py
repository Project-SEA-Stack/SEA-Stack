"""
SEA-Stack external force module client (Python).

Speaks the v1 length-prefixed JSON protocol over TCP to 127.0.0.1.
See docs/extending/EXTERNAL_FORCE_MODULES.md.

Author a 1-DOF PTO (TSDA or RSDA) by subclassing PtoModule and implementing
force(state). Optional setup(cfg) reads config; optional reset / commit /
rollback handle stateful controllers:

    from seastack_external import PtoModule, run

    class MyPTO(PtoModule):
        def setup(self, cfg):
            self.c = float(cfg.get("damping", 50.0))

        def force(self, state):
            return -self.c * state.velocity

    if __name__ == "__main__":
        run(MyPTO())
"""

from __future__ import annotations

import argparse
import json
import socket
import struct
import sys
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple


PROTOCOL_VERSION = 1

# Lean protocol (replay / IPC unit tests): channels 0/1 only.
DEFAULT_PTO_CHANNELS: List[str] = ["displacement", "velocity"]

# Rich schemas published by ExternalPtoModel when EnableRichState is on.
PTO_TSDA_RICH_CHANNELS: List[str] = [
    "displacement",
    "velocity",
    "length",
    "rest_length",
    "rel_accel",
    "body1_pos_x",
    "body1_pos_y",
    "body1_pos_z",
    "body1_vel_x",
    "body1_vel_y",
    "body1_vel_z",
    "body2_pos_x",
    "body2_pos_y",
    "body2_pos_z",
    "body2_vel_x",
    "body2_vel_y",
    "body2_vel_z",
]

PTO_RSDA_RICH_CHANNELS: List[str] = [
    "displacement",
    "velocity",
    "angle",
    "rest_angle",
    "rel_accel",
    "body1_pos_x",
    "body1_pos_y",
    "body1_pos_z",
    "body1_vel_x",
    "body1_vel_y",
    "body1_vel_z",
    "body2_pos_x",
    "body2_pos_y",
    "body2_pos_z",
    "body2_vel_x",
    "body2_vel_y",
    "body2_vel_z",
]


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


def _vec3(
    values: Sequence[float], names: Sequence[str], sx: str, sy: str, sz: str
) -> Tuple[float, float, float]:
    by_name = {n: float(v) for n, v in zip(names, values)}
    return (
        float(by_name.get(sx, 0.0)),
        float(by_name.get(sy, 0.0)),
        float(by_name.get(sz, 0.0)),
    )


@dataclass
class PtoState:
    """Named PTO inputs for force(state) / torque(state) authoring.

    Universal fields (always meaningful when present in the channel list):
      time, dt, displacement, velocity, rel_accel, body1/body2 position & velocity.

    Link-specific (set when the host publishes them via in_names):
      length, rest_length  (TSDA)
      angle, rest_angle    (RSDA)

    Units: m, m/s, N on a TSDA; rad, rad/s, N·m on an RSDA.
    The return value of force() is force [N] or torque [N·m].
    """

    time: float = 0.0
    dt: float = 0.0
    displacement: float = 0.0
    velocity: float = 0.0
    length: float = 0.0
    rest_length: float = 0.0
    angle: float = 0.0
    rest_angle: float = 0.0
    rel_accel: float = 0.0
    body1_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    body1_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    body2_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    body2_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    link_kind: str = ""
    raw: List[float] = field(default_factory=list)
    names: List[str] = field(default_factory=list)

    def get(self, name: str, default: float = 0.0) -> float:
        by_name = {n: float(v) for n, v in zip(self.names, self.raw)}
        return float(by_name.get(name, default))

    @classmethod
    def from_inputs(
        cls,
        t: float,
        dt: float,
        inputs: Sequence[float],
        names: Optional[Sequence[str]] = None,
        link_kind: str = "",
    ) -> "PtoState":
        channel_names = list(names) if names else list(DEFAULT_PTO_CHANNELS)
        # Pad / truncate names to match inputs for zip safety.
        if len(channel_names) < len(inputs):
            channel_names = channel_names + [
                f"in_{i}" for i in range(len(channel_names), len(inputs))
            ]
        elif len(channel_names) > len(inputs):
            channel_names = channel_names[: len(inputs)]
        by_name = {n: float(v) for n, v in zip(channel_names, inputs)}
        disp = float(by_name.get("displacement", inputs[0] if inputs else 0.0))
        vel = float(by_name.get("velocity", inputs[1] if len(inputs) > 1 else 0.0))
        return cls(
            time=float(t),
            dt=float(dt),
            displacement=disp,
            velocity=vel,
            length=float(by_name.get("length", 0.0)),
            rest_length=float(by_name.get("rest_length", 0.0)),
            angle=float(by_name.get("angle", 0.0)),
            rest_angle=float(by_name.get("rest_angle", 0.0)),
            rel_accel=float(by_name.get("rel_accel", 0.0)),
            body1_position=_vec3(
                inputs, channel_names, "body1_pos_x", "body1_pos_y", "body1_pos_z"
            ),
            body1_velocity=_vec3(
                inputs, channel_names, "body1_vel_x", "body1_vel_y", "body1_vel_z"
            ),
            body2_position=_vec3(
                inputs, channel_names, "body2_pos_x", "body2_pos_y", "body2_pos_z"
            ),
            body2_velocity=_vec3(
                inputs, channel_names, "body2_vel_x", "body2_vel_y", "body2_vel_z"
            ),
            link_kind=str(link_kind or ""),
            raw=[float(x) for x in inputs],
            names=list(channel_names),
        )


class ExternalForceModule:
    """Subclass and implement initialize / evaluate / shutdown."""

    # Populated by ExternalForceModule.run() from the initialize handshake.
    _host_in_names: Optional[List[str]] = None
    _host_kind: str = ""

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
                    # Additive host channel names (ignored by older modules).
                    names = msg.get("in_names")
                    if isinstance(names, list):
                        module._host_in_names = [str(x) for x in names]
                    else:
                        module._host_in_names = None
                    module._host_kind = str(msg.get("kind") or "")
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


class PtoModule(ExternalForceModule):
    """1-DOF PTO authoring base: implement force(state) -> float.

    Works for both ChLinkTSDA (force [N]) and ChLinkRSDA (torque [N·m]).
    Optionally override setup(cfg), reset, commit, rollback, shutdown.
    """

    def __init__(self) -> None:
        super().__init__()
        self._channel_names: List[str] = list(DEFAULT_PTO_CHANNELS)
        self._link_kind: str = ""
        self._meta_name: str = self.__class__.__name__
        self._meta_version: str = "1.0"
        self._n_states: int = 0

    def setup(self, cfg: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Optional configuration hook. Return meta overrides if desired."""
        return None

    def force(self, state: PtoState) -> float:
        raise NotImplementedError

    def torque(self, state: PtoState) -> float:
        """Alias for force(); return value is N·m on an RSDA."""
        return self.force(state)

    def initialize(self, cfg: Dict[str, Any]) -> Dict[str, Any]:
        if self._host_in_names:
            self._channel_names = list(self._host_in_names)
        else:
            self._channel_names = list(DEFAULT_PTO_CHANNELS)
        kind = self._host_kind or str(cfg.get("link_kind", "") or "")
        self._link_kind = kind
        meta = self.setup(cfg) or {}
        self._meta_name = str(meta.get("name", self.__class__.__name__))
        self._meta_version = str(meta.get("version", "1.0"))
        self._n_states = int(meta.get("n_states", 0))
        return {
            "name": self._meta_name,
            "version": self._meta_version,
            "n_states": self._n_states,
        }

    def evaluate(self, t: float, dt: float, inputs: List[float]) -> List[float]:
        state = PtoState.from_inputs(
            t, dt, inputs, names=self._channel_names, link_kind=self._link_kind
        )
        return [float(self.force(state))]


def run(module: ExternalForceModule) -> None:
    """Connect `module` to SEA-Stack and serve the IPC loop until shutdown."""
    ExternalForceModule.run(module)


if __name__ == "__main__":
    sys.stderr.write(
        "seastack_external.py is the IPC helper only.\n"
        "Run a demo module instead, e.g.\n"
        "  demos/rm3/external_pto/linear_damper_pto.py\n"
        "  demos/rm3/external_pto_adaptive/adaptive_damping_pto.py\n"
        "  demos/rm3/external_pto_hydraulic/hydraulic_accumulator_pto.py\n"
    )
    sys.exit(2)

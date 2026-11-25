"""Epson SCARA T6-602S controller: picks parts at the conveyor stop and places them in the tray pockets.

PLC -> SCARA (Receiver channel 1, JSON):
  {"cmd": "pick_place", "id": n, "part": DEF, "pick": [x, y, z], "place": [x, y, z]}   (world frame, TCP on part top)
  {"cmd": "hold"} / {"cmd": "release_hold"} / {"cmd": "speed", "value": 0..1}
SCARA -> PLC (Emitter channel 2):
  {"evt": "ready"}, {"evt": "picked", "id"}, {"evt": "placed", "id", "part"}, {"evt": "done", "id"},
  {"evt": "error", "msg"}
The suction cup is modelled kinematically: once vacuum is on, the part follows the TCP (the controller is a supervisor).
"""
import json
import math
import os
import sys

from controller import Supervisor

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "lib"))
import layout as L  # noqa: E402
from motion import Carrier, Executor  # noqa: E402
from scara_kin import fk, ik  # noqa: E402

JOINT_SPEED = 7.0       # rad/s peak-ish (quintic) in joint moves
DOWN_SPEED = 0.6       # m/s for the vertical approach


class Scara(Supervisor):
    def __init__(self):
        super().__init__()
        self.dt = int(self.getBasicTimeStep())
        names = ["base_arm", "arm", "shaft_linear", "shaft_rotation"]
        self.motors = [self.getDevice(n + "_motor") for n in names]
        self.sensors = [self.getDevice(n + "_position") for n in names]
        for s in self.sensors:
            s.enable(self.dt)
        self.rx = self.getDevice("scara_rx")
        self.rx.enable(self.dt)
        self.tx = self.getDevice("scara_tx")
        self.led = self.getDevice("epson_led")
        self.carrier = Carrier(self, "SCARA_TCP")
        bp = self.getSelf().getPosition()
        R = self.getSelf().getOrientation()
        self.base = (bp[0], bp[1], bp[2], math.atan2(R[3], R[0]))
        self.job = None

    def send(self, **m):
        self.tx.send(json.dumps(m))

    def to_base(self, p):
        bx, by, bz, yaw = self.base
        c, s = math.cos(-yaw), math.sin(-yaw)
        u, v = p[0] - bx, p[1] - by
        return (c * u - s * v, s * u + c * v, p[2] - bz, 0.0)

    def _ik(self, t, q):
        r = ik(t[0], t[1], t[2], t[3], L.SCARA_TOOL, [q[0], q[1], q[2], q[3]])
        if r is not None:
            r[3] = 0.0      # parts are round: the shaft does not need to rotate (its motor is slow, 1.8 rad/s)
        return r

    def plan(self, m):
        pick, place = self.to_base(m["pick"]), self.to_base(m["place"])
        clear = L.SCARA_CLEAR_Z - self.base[2]
        up_pick = (pick[0], pick[1], clear, 0.0)
        up_place = (place[0], place[1], clear, 0.0)
        jid, part = m["id"], m["part"]

        def grip():
            self.carrier.grasp([part])
            self.led.set(1)

        def release():
            def settle(n):
                p = n.getPosition()
                n.getField("translation").setSFVec3f([p[0], p[1], m["place"][2] - L.PART_H / 2])
                n.getField("rotation").setSFRotation([0, 0, 1, 0])
            names = self.carrier.release(settle)
            self.led.set(0)
            self.send(evt="placed", id=jid, part=names[0] if names else part)

        def done():
            self.job = None
            self.send(evt="done", id=jid)

        self.job = jid
        self.ex.load([
            ("j", up_pick), ("l", pick, DOWN_SPEED), ("a", grip, 0.12),
            ("l", up_pick, DOWN_SPEED), ("a", lambda: self.send(evt="picked", id=jid), 0.0),
            ("j", up_place), ("l", place, DOWN_SPEED), ("a", release, 0.10),
            ("l", up_place, DOWN_SPEED),
            ("j", up_pick), ("a", done, 0.0),        # wait above the pick: keeps the station clear for the UR5e
        ], self.ex.tcp)

    def poll(self):
        while self.rx.getQueueLength() > 0:
            m = json.loads(self.rx.getString())
            self.rx.nextPacket()
            c = m.get("cmd")
            if c == "hold":
                self.ex.hold = True
            elif c == "release_hold":
                self.ex.hold = False
            elif c == "speed":
                self.ex.override = max(0.05, min(1.0, float(m["value"])))
            elif c == "pick_place" and self.job is None:
                self.plan(m)

    def fk_error(self, q):
        """Distance between our FK (mapped to the world) and the simulator's TCP node, after the move settled."""
        for _ in range(30):
            self.step(self.dt)
        q = [s.getValue() for s in self.sensors]
        x, y, z, _ = fk(q, L.SCARA_TOOL)
        bx, by, bz, yaw = self.base
        c, s_ = math.cos(yaw), math.sin(yaw)
        w = (bx + c * x - s_ * y, by + s_ * x + c * y, bz + z)
        return math.dist(w, self.carrier.tcp.getPosition())

    def run(self):
        self.step(self.dt)
        q0 = [s.getValue() for s in self.sensors]
        self.ex = Executor(self._ik, q0, JOINT_SPEED)
        home = self.to_base((L.PICK[0], L.PICK[1], L.SCARA_CLEAR_Z))
        self.ex.load([("j", (home[0], home[1], L.SCARA_CLEAR_Z - self.base[2], 0.0))], None)
        ready = False
        while self.step(self.dt) != -1:
            self.poll()
            q = self.ex.update(self.dt / 1000.0)
            if self.ex.error:
                self.send(evt="error", msg=self.ex.error)
                self.ex.error = None
            for mtr, v in zip(self.motors, q):
                mtr.setPosition(v)
            self.carrier.update()
            if not ready and not self.ex.busy():
                ready = True
                self.send(evt="ready", fk_err=self.fk_error(q))


Scara().run()

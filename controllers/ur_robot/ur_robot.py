"""UR5e controller: swaps the full tray at the SCARA station for an empty one from the magazine.

PLC -> UR (Receiver channel 3, JSON):
  {"cmd": "swap", "id": n, "full": [DEF tray, DEF parts...], "station": [x, y, z, yaw],
   "cart": [x, y, z, yaw], "mag": [x, y, z, yaw], "empty": DEF tray}        (world frame, TCP = grip point)
  {"cmd": "hold"} / {"cmd": "release_hold"} / {"cmd": "speed", "value": 0..1}
UR -> PLC (Emitter channel 4):
  {"evt": "ready"}, {"evt": "station_free", "id"}, {"evt": "cart_loaded", "id"}, {"evt": "station_loaded", "id"},
  {"evt": "done", "id"}, {"evt": "error", "msg"}
The tray gripper has two real fingers (LinearMotors); the grasp itself is kinematic (the controller is a supervisor).
"""
import json
import math
import os
import sys

from controller import Supervisor

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "lib"))
import layout as L  # noqa: E402
from motion import Carrier, Executor  # noqa: E402
from ur5e_kin import fk, ik_down  # noqa: E402

JOINT_SPEED = 1.6       # rad/s (cobot speeds)
LIN_SPEED = 0.45        # m/s
APPROACH = 0.12         # m/s for the last centimetres
FINGER_OPEN, FINGER_CLOSED = 0.0, 0.017
JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


class UR(Supervisor):
    def __init__(self):
        super().__init__()
        self.dt = int(self.getBasicTimeStep())
        self.motors = [self.getDevice(n) for n in JOINTS]
        self.sensors = [self.getDevice(n + "_sensor") for n in JOINTS]
        for s in self.sensors:
            s.enable(self.dt)
        self.fingers = [self.getDevice("finger_l_motor"), self.getDevice("finger_r_motor")]
        self.rx = self.getDevice("ur_rx")
        self.rx.enable(self.dt)
        self.tx = self.getDevice("ur_tx")
        self.carrier = Carrier(self, "UR_TCP")
        bp = self.getSelf().getPosition()
        R = self.getSelf().getOrientation()
        self.base = (bp[0], bp[1], bp[2], math.atan2(R[3], R[0]))
        self.job = None
        self.finger = FINGER_OPEN

    def send(self, **m):
        self.tx.send(json.dumps(m))

    def to_base(self, p):
        bx, by, bz, yaw = self.base
        c, s = math.cos(-yaw), math.sin(-yaw)
        u, v = p[0] - bx, p[1] - by
        a = p[3] - yaw
        return (c * u - s * v, s * u + c * v, p[2] - bz, math.atan2(math.sin(a), math.cos(a)))

    def _ik(self, t, q):
        return ik_down(t[0], t[1], t[2], t[3], L.UR_TOOL, q)

    def fingers_to(self, v):
        def fn():
            self.finger = v
        return fn

    def plan(self, m):
        jid = m["id"]
        st, cart, mag = (self.to_base(m[k]) for k in ("station", "cart", "mag"))
        home = self.to_base(L.UR_HOME_TCP)
        zc = L.UR_CLEAR_Z - self.base[2]

        def up(p, dz=0.12):
            return (p[0], p[1], max(zc, p[2] + dz), p[3])

        def near(p, dz=0.03):
            return (p[0], p[1], p[2] + dz, p[3])

        def grasp(defs):
            def fn():
                self.carrier.grasp(defs)
            return fn

        def release(evt, tray_z):
            def fn():
                # snap the tray (and its parts) to its exact resting height
                dz = tray_z - self.carrier.held[0][0].getPosition()[2] if self.carrier.held else 0.0

                def settle(n):
                    p = n.getPosition()
                    n.getField("translation").setSFVec3f([p[0], p[1], p[2] + dz])
                self.carrier.release(settle)
                self.send(evt=evt, id=jid)
            return fn

        def done():
            self.job = None
            self.send(evt="done", id=jid)

        # tray origin = bottom centre; the grip point is GRIP_H above it
        cart_c = m["cart"][2] - L.GRIP_H
        st_c = m["station"][2] - L.GRIP_H
        steps = [
            ("a", self.fingers_to(FINGER_OPEN), 0.0),
            ("j", up(st)), ("l", near(st), LIN_SPEED), ("l", st, APPROACH),
            ("a", self.fingers_to(FINGER_CLOSED), 0.35), ("a", grasp(m["full"]), 0.0),
            ("l", up(st), LIN_SPEED), ("a", lambda: self.send(evt="station_free", id=jid), 0.0),
            ("j", up(cart)), ("l", near(cart), LIN_SPEED), ("l", cart, APPROACH),
            ("a", release("cart_loaded", cart_c), 0.0), ("a", self.fingers_to(FINGER_OPEN), 0.3),
            ("l", up(cart), LIN_SPEED),
            ("j", up(mag)), ("l", near(mag), LIN_SPEED), ("l", mag, APPROACH),
            ("a", self.fingers_to(FINGER_CLOSED), 0.35), ("a", grasp([m["empty"]]), 0.0),
            ("l", up(mag), APPROACH * 2),
            ("j", up(st)), ("l", near(st), LIN_SPEED), ("l", st, APPROACH),
            ("a", release("station_loaded", st_c), 0.0), ("a", self.fingers_to(FINGER_OPEN), 0.3),
            ("l", up(st), LIN_SPEED),
            ("j", home), ("a", done, 0.0),
        ]
        self.job = jid
        self.ex.load(steps, self.ex.tcp)

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
            elif c == "swap" and self.job is None:
                self.plan(m)

    def fk_error(self):
        """Distance between our FK (mapped to the world) and the simulator's TCP node, after the move settled."""
        for _ in range(30):
            self.step(self.dt)
        q = [s.getValue() for s in self.sensors]
        T = fk(q, L.UR_TOOL)
        bx, by, bz, yaw = self.base
        c, s_ = math.cos(yaw), math.sin(yaw)
        w = (bx + c * T[0][3] - s_ * T[1][3], by + s_ * T[0][3] + c * T[1][3], bz + T[2][3])
        return math.dist(w, self.carrier.tcp.getPosition())

    def run(self):
        self.step(self.dt)
        q0 = [s.getValue() for s in self.sensors]
        self.ex = Executor(self._ik, q0, JOINT_SPEED)
        home = self.to_base(L.UR_HOME_TCP)
        q_home = self._ik(home, L.UR_Q_HOME)
        self.ex.load([("q", q_home, home)], None)
        ready = False
        while self.step(self.dt) != -1:
            self.poll()
            q = self.ex.update(self.dt / 1000.0)
            if self.ex.error:
                self.send(evt="error", msg=self.ex.error)
                self.ex.error = None
            for mtr, v in zip(self.motors, q):
                mtr.setPosition(v)
            if not self.ex.hold:
                for f in self.fingers:
                    f.setPosition(self.finger)
            self.carrier.update()
            if not ready and not self.ex.busy():
                ready = True
                self.send(evt="ready", fk_err=self.fk_error())


UR().run()

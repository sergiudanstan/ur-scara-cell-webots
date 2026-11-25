"""Operator (Pedestrian PROTO): waits at home; when the PLC sends "walk" (radio channel 5) it walks
layout.OPERATOR_PATH once (to the cart opening and back) with a simple sinusoidal gait."""
import math
import os
import sys

from controller import Supervisor

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "lib"))
import layout as L  # noqa: E402

SPEED = 1.0           # m/s
STRIDE = 1.3          # m per gait cycle
H0 = 1.27

sup = Supervisor()
dt = int(sup.getBasicTimeStep())
me = sup.getSelf()
tr, rot = me.getField("translation"), me.getField("rotation")
rx = sup.getDevice("op_rx")
rx.enable(dt)
J = {n: me.getField(n) for n in ["leftLegAngle", "leftLowerLegAngle", "rightLegAngle", "rightLowerLegAngle",
                                 "leftArmAngle", "rightArmAngle", "leftLowerArmAngle", "rightLowerArmAngle"]}
path, seg, s, phase = None, 0, 0.0, 0.0


def pose(a):
    """Gait at phase a (rad); a = None -> standing."""
    if a is None:
        for f in J.values():
            f.setSFFloat(0.0)
        return 0.0
    hip = 0.45 * math.sin(a)
    J["leftLegAngle"].setSFFloat(-hip)
    J["rightLegAngle"].setSFFloat(hip)
    J["leftLowerLegAngle"].setSFFloat(0.6 * max(0.0, math.sin(a + 1.2)))
    J["rightLowerLegAngle"].setSFFloat(0.6 * max(0.0, math.sin(a + 1.2 + math.pi)))
    J["leftArmAngle"].setSFFloat(0.35 * math.sin(a))
    J["rightArmAngle"].setSFFloat(-0.35 * math.sin(a))
    J["leftLowerArmAngle"].setSFFloat(-0.3)
    J["rightLowerArmAngle"].setSFFloat(-0.3)
    return -0.015 * abs(math.cos(a))


while sup.step(dt) != -1:
    while rx.getQueueLength() > 0:
        msg = rx.getString()
        rx.nextPacket()
        if path is None and msg == "walk":
            path, seg, s = L.OPERATOR_PATH, 0, 0.0
    if path is None:
        pose(None)
        continue
    a, b = path[seg], path[seg + 1]
    seg_len = math.dist(a, b)
    s += SPEED * dt / 1000.0
    phase += 2 * math.pi * SPEED * dt / 1000.0 / STRIDE
    if s >= seg_len:
        s, seg = s - seg_len, seg + 1
        if seg >= len(path) - 1:
            path = None
            pose(None)
            continue
        a, b = path[seg], path[seg + 1]
        seg_len = math.dist(a, b)
    k = s / seg_len
    dz = pose(phase)
    tr.setSFVec3f([a[0] + (b[0] - a[0]) * k, a[1] + (b[1] - a[1]) * k, H0 + dz])
    rot.setSFRotation([0, 0, 1, math.atan2(b[1] - a[1], b[0] - a[0])])

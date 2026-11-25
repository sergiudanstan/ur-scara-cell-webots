"""Offline kinematics tests: IK round-trips through FK, and every cell target is reachable.
Run: python3 -m pytest tests  (or python3 tests/test_kinematics.py)
"""
import math
import os
import sys

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(ROOT, "controllers", "scara_robot"))
sys.path.insert(0, os.path.join(ROOT, "controllers", "ur_robot"))
sys.path.insert(0, os.path.join(ROOT, "lib"))

import scara_kin  # noqa: E402
import ur5e_kin  # noqa: E402
import layout  # noqa: E402


def test_scara_roundtrip():
    for x, y, z, yaw in [(0.45, 0.2, 0.0, 0.3), (0.3, -0.35, -0.05, -1.0), (0.55, 0.0, 0.03, 0.0)]:
        q = scara_kin.ik(x, y, z, yaw, layout.SCARA_TOOL)
        assert q is not None, (x, y, z)
        p = scara_kin.fk(q, layout.SCARA_TOOL)
        assert max(abs(a - b) for a, b in zip(p, (x, y, z, yaw))) < 1e-9, (p, (x, y, z, yaw))


def test_scara_rejects_out_of_reach():
    assert scara_kin.ik(0.9, 0.0, 0.0, 0.0, layout.SCARA_TOOL) is None
    assert scara_kin.ik(0.4, 0.0, 0.5, 0.0, layout.SCARA_TOOL) is None


def _ur_check(x, y, z, yaw, seed):
    q = ur5e_kin.ik_down(x, y, z, yaw, layout.UR_TOOL, seed)
    assert q is not None, (x, y, z, yaw)
    T = ur5e_kin.fk(q, layout.UR_TOOL)
    assert max(abs(T[i][3] - v) for i, v in enumerate((x, y, z))) < 1e-5
    assert abs(T[2][1] + 1) < 1e-5            # tool axis points down
    return q


def test_ur_roundtrip():
    seed = layout.UR_Q_HOME
    for tgt in [(0.4, 0.3, 0.2, 0.0), (-0.3, 0.5, 0.0, 1.2), (0.5, -0.2, -0.1, -0.5)]:
        seed = _ur_check(*tgt, seed)


def _scara_base(p):
    bx, by, bz, byaw = layout.SCARA_BASE
    c, s = math.cos(-byaw), math.sin(-byaw)
    u, v = p[0] - bx, p[1] - by
    return c * u - s * v, s * u + c * v, p[2] - bz


def _ur_base(p):
    bx, by, bz, byaw = layout.UR_BASE
    c, s = math.cos(-byaw), math.sin(-byaw)
    u, v = p[0] - bx, p[1] - by
    return c * u - s * v, s * u + c * v, p[2] - bz


def test_cell_targets_reachable():
    # SCARA: conveyor pick, every tray pocket, transfer height
    for p in [layout.PICK] + layout.pocket_positions():
        for z in (p[2], layout.SCARA_CLEAR_Z):
            x, y, zb = _scara_base((p[0], p[1], z))
            assert scara_kin.ik(x, y, zb, 0.0, layout.SCARA_TOOL) is not None, (p, z)
    # UR: tray station, magazine (all levels), every cart slot, at grip and clearance heights
    seed = layout.UR_Q_HOME
    targets = [layout.station_grip()] + [layout.magazine_grip(k) for k in range(layout.MAG_COUNT)] + \
              [layout.cart_grip(k) for k in range(layout.CART_SLOTS)]
    for (x, y, z, yaw) in targets:
        for zz in (z, z + 0.12, layout.UR_CLEAR_Z):
            xb, yb, zb = _ur_base((x, y, zz))
            seed = _ur_check(xb, yb, zb, yaw - layout.UR_BASE[3], seed)


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)


def test_ur_swap_cycles_without_ik_errors():
    """Replays the UR5e swap program for every cart slot / magazine level with the real motion executor."""
    sys.path.insert(0, os.path.join(ROOT, "lib"))
    from motion import Executor
    bx, by, bz, byaw = layout.UR_BASE

    def tb(p):
        x, y, z = _ur_base(p[:3])
        a = p[3] - byaw
        return (x, y, z, math.atan2(math.sin(a), math.cos(a)))

    def ik(t, q):
        return ur5e_kin.ik_down(t[0], t[1], t[2], t[3], layout.UR_TOOL, q)

    home = tb(layout.UR_HOME_TCP)
    ex = Executor(ik, ik(home, layout.UR_Q_HOME), 1.6)
    ex.tcp = home
    zc = layout.UR_CLEAR_Z - bz

    def up(p):
        return (p[0], p[1], max(zc, p[2] + 0.12), p[3])

    for k in range(layout.CART_SLOTS):
        st, cart, mag = tb(layout.station_grip()), tb(layout.cart_grip(k)), tb(layout.magazine_grip(k))
        prog = []
        for p in (st, cart, mag, st):
            prog += [("j", up(p)), ("l", (p[0], p[1], p[2] + 0.03, p[3]), 0.45), ("l", p, 0.12), ("l", up(p), 0.45)]
        ex.load(prog + [("j", home)], ex.tcp)
        prev = list(ex.q)
        while ex.busy():
            ex.update(0.016)
            assert ex.error is None, (k, ex.error)
            # joint speed below the UR5e limits (3.14 rad/s base joints)
            assert max(abs(a - b) for a, b in zip(prev, ex.q)) / 0.016 < 3.1
            prev = list(ex.q)

"""Forward / inverse kinematics of the Universal Robots UR5e, as modelled in Webots R2025a (UR5e.proto).

The FK is the PROTO joint chain itself (endPoint translations and fixed rotations), so it matches the simulator
exactly; the IK is a damped least-squares solver on the 6-D pose error. The tool axis is +y of the wrist-3 frame
(the PROTO's toolSlot sits 0.1 m along it). No dependencies.
"""
import math

LIMITS = [(-6.28, 6.28), (-6.28, 6.28), (-3.14, 3.14), (-6.28, 6.28), (-6.28, 6.28), (-6.28, 6.28)]
TOOL_SLOT = 0.1


def _m(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _t(x, y, z):
    return [[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]]


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]]


def fk(q, tool_len):
    """4x4 pose of the TCP in the robot base frame. Column 1 (y) is the tool approach axis."""
    T = _m(_t(0, 0, 0.163), _rz(q[0]))
    T = _m(T, _m(_t(0, 0.138, 0), _ry(q[1] + math.pi / 2)))
    T = _m(T, _m(_t(0, -0.131, 0.425), _ry(q[2])))
    T = _m(T, _m(_t(0, 0, 0.392), _ry(q[3] + math.pi / 2)))
    T = _m(T, _m(_t(0, 0.127, 0), _rz(q[4])))
    T = _m(T, _m(_t(0, 0, 0.1), _ry(q[5])))
    return _m(T, _t(0, TOOL_SLOT + tool_len, 0))


def target_down(x, y, z, yaw):
    """Pose with the tool pointing down (y_tool = -z_world) and the gripper's closing axis x_tool at angle yaw."""
    c, s = math.cos(yaw), math.sin(yaw)
    return [[c, 0, -s, x], [s, 0, c, y], [0, -1, 0, z], [0, 0, 0, 1]]


def _err(T, Tt):
    e = [Tt[i][3] - T[i][3] for i in range(3)]
    # orientation error = log map of Rt * R^T (rotation vector, world frame)
    Re = [[sum(Tt[i][k] * T[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
    c = max(-1.0, min(1.0, (Re[0][0] + Re[1][1] + Re[2][2] - 1) / 2))
    ang = math.acos(c)
    v = [Re[2][1] - Re[1][2], Re[0][2] - Re[2][0], Re[1][0] - Re[0][1]]
    s = math.sin(ang)
    if ang < 1e-9:
        w = [0.5 * x for x in v]
    elif s > 1e-6:
        w = [x * ang / (2 * s) for x in v]
    else:  # ~180 deg: axis from the diagonal
        i = max(range(3), key=lambda k: Re[k][k])
        ax = [0.0, 0.0, 0.0]
        ax[i] = math.sqrt(max(0.0, (Re[i][i] + 1) / 2))
        for j in range(3):
            if j != i:
                ax[j] = (Re[i][j] + Re[j][i]) / (4 * ax[i])
        w = [x * ang for x in ax]
    return e + w


def _solve(M, v):
    n = len(v)
    A = [row[:] + [v[i]] for i, row in enumerate(M)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(A[r][c]))
        A[c], A[p] = A[p], A[c]
        for r in range(n):
            if r != c:
                f = A[r][c] / A[c][c]
                for k in range(c, n + 1):
                    A[r][k] -= f * A[c][k]
    return [A[i][n] / A[i][i] for i in range(n)]


def _in_limits(q, q_seed):
    """Shift each joint by multiples of 2*pi towards the seed; None if a joint cannot be brought inside its limits."""
    out = []
    for a, s, (lo, hi) in zip(q, q_seed, LIMITS):
        a += 2 * math.pi * round((s - a) / (2 * math.pi))
        while a > hi:
            a -= 2 * math.pi
        while a < lo:
            a += 2 * math.pi
        if a > hi:
            return None
        out.append(a)
    return out


def ik(Tt, tool_len, q_seed, iters=200, tol=1e-6):
    """Damped least squares from q_seed. Returns q or None if it does not converge."""
    q = list(q_seed)
    lam = 1e-3
    for _ in range(iters):
        T = fk(q, tool_len)
        e = _err(T, Tt)
        if max(abs(x) for x in e) < tol:
            return _in_limits(q, q_seed)
        J = []
        h = 1e-6
        for k in range(6):
            dq = list(q)
            dq[k] += h
            ek = _err(fk(dq, tool_len), Tt)
            J.append([(e[i] - ek[i]) / h for i in range(6)])   # column k of d(pose)/dq
        JJt = [[sum(J[k][i] * J[k][j] for k in range(6)) + (lam if i == j else 0) for j in range(6)] for i in range(6)]
        y = _solve(JJt, e)
        step = [sum(J[k][i] * y[i] for i in range(6)) for k in range(6)]
        n = max(abs(s) for s in step)
        if n > 0.3:
            step = [s * 0.3 / n for s in step]
        q = [a + s for a, s in zip(q, step)]
    return None


REST = [-1.9, 1.65, -1.3, -1.5708, 0.0]      # shoulder-lift .. wrist-3 of an elbow-up, tool-down posture


def ik_down(x, y, z, yaw, tool_len, q_seed):
    """Tool-down IK. Tries the seed first, then elbow-up seeds aimed at the target; keeps the closest solution."""
    Tt = target_down(x, y, z, yaw)
    q = ik(Tt, tool_len, q_seed)
    if q is not None:
        return q
    best = None
    for k in range(4):
        seed = [math.atan2(y, x) + k * math.pi / 2] + REST
        q = ik(Tt, tool_len, seed)
        if q is not None:
            q = _in_limits(q, q_seed)
        if q is not None:
            d = sum(abs(a - b) for a, b in zip(q, q_seed))
            if best is None or d < best[0]:
                best = (d, q)
    return None if best is None else best[1]

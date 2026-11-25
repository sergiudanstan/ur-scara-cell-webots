"""Shared motion executor for both robot controllers.

A job is a list of steps executed in order:
  ("q", q, target)         joint-interpolated move to explicit joint values q (target = the TCP they give)
  ("j", target)            joint-interpolated move to a TCP target (x, y, z, yaw) in the robot base frame
  ("l", target, speed)     straight-line TCP move (quintic profile, IK at every control step)
  ("a", fn, dwell)         one-shot action (grip / release / message), then wait `dwell` seconds
The executor is paused while `hold` is set (protective stop, category 2: the robot keeps its position under servo)
and every duration is stretched by 1/override (speed override from the PLC).
"""
import math


def quintic(s):
    s = max(0.0, min(1.0, s))
    return s * s * s * (10 - 15 * s + 6 * s * s)


def mat3(o):
    return [[o[0], o[1], o[2]], [o[3], o[4], o[5]], [o[6], o[7], o[8]]]


def mul3(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def tr3(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


def mv3(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def axis_angle(m):
    c = max(-1.0, min(1.0, (m[0][0] + m[1][1] + m[2][2] - 1) / 2))
    ang = math.acos(c)
    if ang < 1e-6:
        return [0, 0, 1, 0]
    s = math.sin(ang)
    if s > 1e-4:
        return [(m[2][1] - m[1][2]) / (2 * s), (m[0][2] - m[2][0]) / (2 * s), (m[1][0] - m[0][1]) / (2 * s), ang]
    i = max(range(3), key=lambda k: m[k][k])
    ax = [0.0, 0.0, 0.0]
    ax[i] = math.sqrt(max(0.0, (m[i][i] + 1) / 2))
    for j in range(3):
        if j != i:
            ax[j] = (m[i][j] + m[j][i]) / (4 * ax[i])
    return ax + [ang]


class Executor:
    def __init__(self, ik, q0, joint_speed, min_t=0.4):
        """ik(target, q_ref) -> q or None."""
        self.ik, self.q, self.joint_speed, self.min_t = ik, list(q0), joint_speed, min_t
        self.steps, self.cur, self.tcp = [], None, None
        self.hold, self.override, self.error = False, 1.0, None

    def busy(self):
        return self.cur is not None or bool(self.steps)

    def load(self, steps, tcp_now):
        self.steps, self.cur, self.tcp = list(steps), None, tcp_now

    def _start(self):
        st = self.steps.pop(0)
        if getattr(self, "debug", False):
            print("step", st[0], [round(v, 3) for v in st[1]] if st[0] != "a" else getattr(st[1], "__name__", ""))
        if st[0] == "j":
            q1 = self.ik(st[1], self.q)
            if q1 is None:
                self.error = "unreachable %s" % (st[1],)
                q1 = self.q
            dq = max(abs(a - b) for a, b in zip(self.q, q1))
            T = max(self.min_t, 1.875 * dq / self.joint_speed)
            self.cur = dict(kind="j", q0=list(self.q), q1=q1, T=T, t=0.0)
            self.tcp = st[1]
        elif st[0] == "q":      # explicit joint target (q, tcp it corresponds to)
            dq = max(abs(a - b) for a, b in zip(self.q, st[1]))
            T = max(self.min_t, 1.875 * dq / self.joint_speed)
            self.cur = dict(kind="j", q0=list(self.q), q1=list(st[1]), T=T, t=0.0)
            self.tcp = st[2]
        elif st[0] == "l":
            p0 = self.tcp or st[1]
            d = math.dist(p0[:3], st[1][:3])
            T = max(0.25, 1.875 * d / st[2])
            self.cur = dict(kind="l", p0=list(p0), p1=list(st[1]), T=T, t=0.0)
            self.tcp = st[1]
        else:
            st[1]()
            self.cur = dict(kind="a", T=st[2], t=0.0)

    def update(self, dt):
        """Advance by dt seconds and return the joint command."""
        if self.hold:
            return self.q
        if self.cur is None and self.steps:
            self._start()
        c = self.cur
        if c is None:
            return self.q
        c["t"] += dt * self.override
        s = quintic(c["t"] / c["T"]) if c["T"] > 0 else 1.0
        if c["kind"] == "j":
            self.q = [a + (b - a) * s for a, b in zip(c["q0"], c["q1"])]
        elif c["kind"] == "l":
            p = [a + (b - a) * s for a, b in zip(c["p0"], c["p1"])]
            q = self.ik(p, self.q)
            if q is None:
                self.error = "linear move left the workspace at %s" % (p,)
            else:
                self.q = q
        if c["t"] >= c["T"]:
            self.cur = None
        return self.q


class Carrier:
    """Kinematic grasp: objects follow the TCP node while held (the controller must be a supervisor)."""

    def __init__(self, sup, tcp_def):
        self.sup, self.tcp = sup, sup.getFromDef(tcp_def)
        self.held = []

    def tcp_pose(self):
        return self.tcp.getPosition(), mat3(self.tcp.getOrientation())

    def grasp(self, defs):
        P, R = self.tcp_pose()
        if getattr(self, "debug", False):
            print("grasp", defs[:1], [round(v, 3) for v in P], [[round(v, 3) for v in r] for r in R])
        Rt = tr3(R)
        self.held = []
        for d in defs:
            n = self.sup.getFromDef(d)
            if n is None:
                continue
            p = n.getPosition()
            self.held.append((n, mv3(Rt, [p[i] - P[i] for i in range(3)]), mul3(Rt, mat3(n.getOrientation()))))

    def update(self):
        if not self.held:
            return
        P, R = self.tcp_pose()
        for n, rp, rr in self.held:
            w = mv3(R, rp)
            n.getField("translation").setSFVec3f([P[0] + w[0], P[1] + w[1], P[2] + w[2]])
            n.getField("rotation").setSFRotation(axis_angle(mul3(R, rr)))

    def release(self, settle=None):
        """settle(node) may snap the final pose (exact height, yaw rounded)."""
        names = []
        if getattr(self, "debug", False):
            P, R = self.tcp_pose()
            print("release", [round(v, 3) for v in P], [[round(v, 3) for v in r] for r in R])
        for n, _, _ in self.held:
            if settle:
                settle(n)
            names.append(n.getDef())
        self.held = []
        return names

"""Forward / inverse kinematics of the Epson SCARA T6-602S, as modelled in Webots R2025a (ScaraT6.proto).

Offsets are read from the HingeJoint/SliderJoint chain of the PROTO (robot base frame, metres):
  J1 axis at (0.06, 0); J2 axis at (0.370, 0.005) in the link-1 frame;
  shaft axis at (0.2615, 0) in the arm frame, whose J2 anchor is (0.007, 0.005)  ->  link-2 vector (0.2545, -0.005).
  Hand slot height = 0.256 - 0.074 + d,  d = shaft_linear_motor position in [-0.200, 0].
The tool (suction cup) hangs TOOL_LEN below the hand slot. Coordinates passed in and out are in the robot base frame.
No dependencies.
"""
import math

J1 = (0.06, 0.0)
A = (0.31, 0.005)            # J1 -> J2
B = (0.2545, -0.005)         # J2 -> shaft
HAND_Z0 = 0.182              # hand-slot height at d = 0
D_MIN, D_MAX = -0.200, 0.0
LIMITS = [(-2.30, 2.30), (-2.60, 2.60), (D_MIN, D_MAX), (-6.28, 6.28)]

_LA, _LB = math.hypot(*A), math.hypot(*B)
_AA, _AB = math.atan2(A[1], A[0]), math.atan2(B[1], B[0])
R_MIN, R_MAX = abs(_LA - _LB), _LA + _LB


def fk(q, tool_len):
    """q = [q1, q2, d, q4] -> (x, y, z, yaw) of the TCP in the base frame."""
    q1, q2, d, q4 = q
    c1, s1 = math.cos(q1), math.sin(q1)
    c12, s12 = math.cos(q1 + q2), math.sin(q1 + q2)
    x = J1[0] + c1 * A[0] - s1 * A[1] + c12 * B[0] - s12 * B[1]
    y = J1[1] + s1 * A[0] + c1 * A[1] + s12 * B[0] + c12 * B[1]
    return x, y, HAND_Z0 + d - tool_len, q1 + q2 + q4


def ik(x, y, z, yaw, tool_len, q_ref=None):
    """TCP target in base frame -> [q1, q2, d, q4], or None when out of reach / limits.
    Picks the elbow solution closest to q_ref (default: right-handed elbow)."""
    q_ref = q_ref or [0.0, 0.5, 0.0, 0.0]
    d = z + tool_len - HAND_Z0
    if not D_MIN - 1e-9 <= d <= D_MAX + 1e-9:
        return None
    u, v = x - J1[0], y - J1[1]
    r2 = u * u + v * v
    c = (r2 - _LA ** 2 - _LB ** 2) / (2 * _LA * _LB)
    if abs(c) > 1.0:
        return None
    best = None
    for sgn in (1, -1):
        t2 = sgn * math.acos(c)                       # angle between link vectors
        t1 = math.atan2(v, u) - math.atan2(_LB * math.sin(t2), _LA + _LB * math.cos(t2))
        q1 = t1 - _AA
        q2 = t2 + _AA - _AB
        q1 = math.atan2(math.sin(q1), math.cos(q1))
        q2 = math.atan2(math.sin(q2), math.cos(q2))
        q4 = yaw - q1 - q2
        q4 += 2 * math.pi * round((q_ref[3] - q4) / (2 * math.pi))
        q = [q1, q2, d, q4]
        if all(lo - 1e-9 <= qi <= hi + 1e-9 for qi, (lo, hi) in zip(q, LIMITS)):
            cost = abs(q1 - q_ref[0]) + abs(q2 - q_ref[1])
            if best is None or cost < best[0]:
                best = (cost, q)
    return None if best is None else [min(max(v_, lo), hi) for v_, (lo, hi) in zip(best[1], LIMITS)]

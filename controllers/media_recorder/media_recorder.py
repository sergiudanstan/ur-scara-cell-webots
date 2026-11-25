"""Media recorder (supervisor added to a copy of the world by tools/build_media.sh).

controllerArgs:
  --out=<dir>                          output folder
  --shot=<t>:<name>:<eye x,y,z>:<target x,y,z>        still image (PNG) at simulated time t   (repeatable)
  --movie=<name>:<t0>:<t1>:<eye>:<target>[:<eye1>:<target1>]  MP4 from t0 to t1, camera moving eye -> eye1
  --keys=<t>:<letters>                  PLC commands at time t (written into the PLC customData)  (repeatable)
  --quit=<t>                            quit at t
"""
import math
import os
import sys

from controller import Supervisor


def vec(s):
    return [float(v) for v in s.split(",")]


def look(eye, tgt):
    dx, dy, dz = (tgt[i] - eye[i] for i in range(3))
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.hypot(dx, dy))
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    R = [[cy * cp, -sy, cy * sp], [sy * cp, cy, sy * sp], [-sp, 0, cp]]      # Rz(yaw) * Ry(pitch)
    c = max(-1.0, min(1.0, (R[0][0] + R[1][1] + R[2][2] - 1) / 2))
    a = math.acos(c)
    if a < 1e-6:
        return [0, 0, 1, 0]
    s = 2 * math.sin(a)
    return [(R[2][1] - R[1][2]) / s, (R[0][2] - R[2][0]) / s, (R[1][0] - R[0][1]) / s, a]


sup = Supervisor()
dt = int(sup.getBasicTimeStep())
args = {"shot": [], "keys": []}
for a in sys.argv[1:]:
    k, v = a[2:].split("=", 1)
    if k in ("shot", "keys"):
        args[k].append(v)
    else:
        args[k] = v
out = args.get("out", ".")
os.makedirs(out, exist_ok=True)
view = sup.getFromDef("VIEW")
plc = None
for i in range(sup.getRoot().getField("children").getCount()):
    n = sup.getRoot().getField("children").getMFNode(i)
    if n.getTypeName() == "Robot" and n.getField("name").getSFString() == "cell PLC":
        plc = n
shots = []
for s in args["shot"]:
    t, name, eye, tgt = s.split(":")
    shots.append((float(t), name, vec(eye), vec(tgt)))
shots.sort()
keys = sorted((float(s.split(":")[0]), s.split(":")[1]) for s in args["keys"])
movie = None
if "movie" in args:
    p = args["movie"].split(":")
    movie = dict(name=p[0], t0=float(p[1]), t1=float(p[2]), e0=vec(p[3]), g0=vec(p[4]),
                 e1=vec(p[5]) if len(p) > 5 else vec(p[3]), g1=vec(p[6]) if len(p) > 6 else vec(p[4]), on=False)
t_quit = float(args.get("quit", 1e9))


def set_view(eye, tgt):
    view.getField("position").setSFVec3f(eye)
    view.getField("orientation").setSFRotation(look(eye, tgt))


while sup.step(dt) != -1:
    t = sup.getTime()
    while keys and t >= keys[0][0]:
        plc.getField("customData").setSFString(keys.pop(0)[1])
    if movie:
        if not movie["on"] and t >= movie["t0"]:
            set_view(movie["e0"], movie["g0"])
            sup.movieStartRecording(os.path.join(out, movie["name"]), 1280, 720, 0, 92, 1, False)
            movie["on"] = True
        if movie["on"]:
            k = min(1.0, (t - movie["t0"]) / (movie["t1"] - movie["t0"]))
            k = k * k * (3 - 2 * k)
            set_view([a + (b - a) * k for a, b in zip(movie["e0"], movie["e1"])],
                     [a + (b - a) * k for a, b in zip(movie["g0"], movie["g1"])])
            if t >= movie["t1"]:
                sup.movieStopRecording()
                while not sup.movieIsReady():
                    sup.step(dt)
                print("movie", os.path.join(out, movie["name"]), "failed" if sup.movieFailed() else "ok")
                movie = None
    if shots and t >= shots[0][0]:
        _, name, eye, tgt = shots.pop(0)
        set_view(eye, tgt)
        sup.step(dt)
        sup.step(dt)
        sup.exportImage(os.path.join(out, name), 95)
        print("shot", os.path.join(out, name))
    if t >= t_quit or (not shots and movie is None and not keys and "quit" not in args):
        break
for _ in range(5):
    sup.step(dt)
sup.simulationQuit(0)

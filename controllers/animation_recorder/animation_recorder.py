"""Records the cell as a Webots HTML5 animation (interactive 3D scene for a browser), then quits.
Added to a copy of the world by tools/build_site.sh.   controllerArgs: <out.html> <t_start> <t_end>"""
import sys

from controller import Supervisor

sup = Supervisor()
dt = int(sup.getBasicTimeStep())
out, t0, t1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
recording = False
while sup.step(dt) != -1:
    t = sup.getTime()
    if not recording and t >= t0:
        sup.animationStartRecording(out)
        recording = True
    if recording and t >= t1:
        sup.animationStopRecording()
        sup.step(dt)
        break
sup.simulationQuit(0)

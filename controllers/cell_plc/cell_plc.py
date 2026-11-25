"""Cell PLC (supervisor): sequencing, safety, and the simulated process (belt, cart, door, magazine refill).

CONTROL part: decisions use the Webots sensors B1 (part at the stop), B5 (accumulation full), B2 (tray at the station),
B7 (cart docked), the area scanner (operator distance) and the robots' events.
PROCESS part (simulated): belt transport with accumulation, cart trip, door swing, operator refilling the magazine.

Keys (click the 3D view first) or the PLC robot's customData field:
  E  emergency stop (press / release)      D  open / close the guard door      R  reset
  P  send the operator to the cart opening (speed-and-separation monitoring of the UR5e)
  S  end-of-cycle stop / restart

controllerArgs "--smoke=<s>": scripted self-test (cycle counts, FK vs simulator, safety reactions), then quit with
status 0 (pass) or 1 (fail). Used by tests/run_smoke.sh.
"""
import json
import math
import os
import sys
import time

from controller import Keyboard, Supervisor

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "lib"))
import layout as L  # noqa: E402

N_PARTS, N_TRAYS = 64, L.MAG_COUNT + 1
CART_SPEED = 0.5
CART_AWAY_T = 4.0
SCAN_CLEAR_T = 1.5
LAMP = {"RED": "0.9 0.1 0.08", "AMBER": "1 0.6 0.05", "GREEN": "0.1 0.85 0.2"}


class PLC(Supervisor):
    def __init__(self):
        super().__init__()
        self.dt = int(self.getBasicTimeStep())
        self.sens = {}
        for n in ("B1", "B5", "B2", "B7"):
            d = self.getDevice(n)
            d.enable(self.dt)
            self.sens[n] = d
        self.tx_s, self.tx_u = self.getDevice("to_scara"), self.getDevice("to_ur")
        self.tx_op = self.getDevice("to_operator")
        self.rx_s, self.rx_u = self.getDevice("from_scara"), self.getDevice("from_ur")
        self.rx_s.enable(self.dt)
        self.rx_u.enable(self.dt)
        self.kb = self.getKeyboard()
        self.kb.enable(self.dt)
        self.me = self.getSelf()
        self.node = {d: self.getFromDef(d) for d in
                     ["PART_%d" % i for i in range(N_PARTS)] + ["TRAY_%d" % i for i in range(N_TRAYS)] +
                     ["CART", "DOOR", "OPERATOR"]}
        self.lamp = {k: self.getFromDef("LAMP_" + k).getField("emissiveColor") for k in LAMP}
        # process state
        self.pool = ["PART_%d" % i for i in range(N_PARTS)]
        self.belt = []                      # [[DEF, x]] sorted, front first
        self.magazine = ["TRAY_%d" % i for i in range(L.MAG_COUNT)]   # bottom -> top
        self.station = {"tray": "TRAY_%d" % L.MAG_COUNT, "parts": []}
        self.cart_load = []                 # [(tray, parts)]
        self.cart = {"state": "docked", "y": L.CART[1], "t": 0.0}
        self.last_feed = -10.0
        self.door_open, self.door_angle = False, 0.0
        # control state
        self.scara = {"ready": False, "busy": False, "id": 0, "part": None, "pocket": 0}
        self.ur = {"ready": False, "busy": False, "id": 0}
        self.estop = False
        self.fault = None                   # latched cause, cleared by R
        self.cycle_stop = False
        self.scan_stop, self.scan_warn, self.scan_clear_t = False, False, 0.0
        self.held = False
        self.override = None
        self.events = []
        self.stats = {"parts": 0, "trays": 0, "carts": 0, "errors": []}
        self.fk_check = {}
        self.smoke = None
        for a in sys.argv[1:]:
            if a.startswith("--smoke="):
                self.smoke = float(a.split("=", 1)[1])
        self.op = self.node["OPERATOR"]

    # -------------------------------------------------------------- helpers
    def log(self, msg):
        t = self.getTime()
        self.events.append("%6.1f  %s" % (t, msg))
        self.events = self.events[-8:]
        print("[PLC %6.1f] %s" % (t, msg))

    def put(self, d, p, yaw=0.0):
        n = self.node[d]
        n.getField("translation").setSFVec3f(list(p))
        n.getField("rotation").setSFRotation([0, 0, 1, yaw])

    def sees(self, name, limit=0.3):
        return self.sens[name].getValue() < limit

    def send_scara(self, **m):
        self.tx_s.send(json.dumps(m))

    def send_ur(self, **m):
        self.tx_u.send(json.dumps(m))

    # -------------------------------------------------------------- process simulation
    def run_belt(self, dt):
        run = not self.cycle_stop and self.cell_ok()
        if run:
            limit = L.PICK[0]
            for item in self.belt:
                if not (item[0] == self.scara["part"] and self.scara.get("gripped")):
                    item[1] = min(item[1] + L.BELT_SPEED * dt, limit)
                limit = item[1] - L.PART_GAP
            entry_free = all(x > L.SPAWN_X + L.PART_GAP for _, x in self.belt)
            if (self.getTime() - self.last_feed >= L.FEED_PERIOD and entry_free and not self.sees("B5")
                    and self.pool):
                d = self.pool.pop(0)
                self.belt.append([d, L.SPAWN_X])
                self.last_feed = self.getTime()
        for d, x in self.belt:
            if not (d == self.scara["part"] and self.scara.get("gripped")):
                self.put(d, (x, L.CONV_Y, L.CONV_Z + L.PART_H / 2))

    def run_cart(self, dt):
        c = self.cart
        moving = self.cell_ok() and not self.scan_stop
        if c["state"] == "leaving" and moving:
            c["y"] = max(L.CART_OUT[1], c["y"] - CART_SPEED * dt)
            if c["y"] <= L.CART_OUT[1]:
                c["state"], c["t"] = "away", self.getTime()
                self.unload_cart()
        elif c["state"] == "away" and self.getTime() - c["t"] > CART_AWAY_T:
            c["state"] = "returning"
            self.log("Empty cart returning")
        elif c["state"] == "returning" and moving:
            c["y"] = min(L.CART[1], c["y"] + CART_SPEED * dt)
            if c["y"] >= L.CART[1]:
                c["state"] = "docked"
                self.log("Cart docked")
        if c["state"] != "docked":
            dy = c["y"] - L.CART[1]
            self.put("CART", (L.CART[0], c["y"], 0))
            for k, (tray, parts) in enumerate(self.cart_load):
                g = L.cart_grip(k)
                self.put(tray, (g[0], g[1] + dy, g[2] - L.GRIP_H))
                for j, pd in enumerate(parts):
                    px, py = L.POCKETS[j]
                    self.put(pd, (g[0] + px, g[1] + py + dy, g[2] - L.GRIP_H + L.TRAY_FLOOR + L.PART_H / 2))

    def unload_cart(self):
        """Out of the cell: trays are emptied and stacked back in the magazine by the operator."""
        self.stats["carts"] += 1
        for k, (tray, parts) in enumerate(self.cart_load):
            for j, pd in enumerate(parts):
                self.put(pd, (-3.0 + 0.06 * j, -4.0 - 0.06 * k, -0.5))
                self.pool.append(pd)
            self.magazine.append(tray)
        self.cart_load = []
        for i, tray in enumerate(self.magazine):
            self.put(tray, (L.MAG[0], L.MAG[1], L.TABLE_Z + i * L.TRAY_H))
        self.log("Cart unloaded, magazine refilled (%d trays)" % len(self.magazine))

    def run_door(self, dt):
        tgt = 1.6 if self.door_open else 0.0
        step = 1.2 * dt
        self.door_angle += max(-step, min(step, tgt - self.door_angle))
        dx_, _, dwid = L.DOOR
        self.node["DOOR"].getField("rotation").setSFRotation([0, 0, 1, self.door_angle])

    # -------------------------------------------------------------- safety
    def operator_distance(self):
        p = self.op.getPosition()
        if p[1] > L.SCANNER[1]:          # the scanner looks south only (fields = half discs outside the fence)
            return float("inf")
        return math.hypot(p[0] - L.SCANNER[0], p[1] - L.SCANNER[1])

    def cell_ok(self):
        return not self.estop and self.fault is None and not self.door_open

    def safety(self):
        if self.estop and self.fault is None:
            self.fault = "EMERGENCY STOP"
            self.log("Emergency stop pressed")
        if self.door_open and self.fault is None:
            self.fault = "GUARD DOOR OPEN"
            self.log("Guard door opened: cell stopped")
        d = self.operator_distance()
        inside_stop, inside_warn = d < L.STOP_R, d < L.WARN_R
        if inside_stop:
            if not self.scan_stop:
                self.log("Scanner protective field: robots hold (cat. 2)")
            self.scan_stop, self.scan_clear_t = True, self.getTime()
        elif self.scan_stop and self.getTime() - self.scan_clear_t > SCAN_CLEAR_T:
            self.scan_stop = False
            self.log("Protective field clear: automatic restart")
        if inside_warn != self.scan_warn:
            self.log("Scanner warning field %s" % ("entered: UR5e at reduced speed" if inside_warn else "clear"))
        self.scan_warn = inside_warn
        hold = not self.cell_ok() or self.scan_stop
        if hold != self.held:
            self.held = hold
            for s in (self.send_scara, self.send_ur):
                s(cmd="hold" if hold else "release_hold")
        ov = (L.UR_SLOW if self.scan_warn else 1.0, 0.5 if self.scan_warn else 1.0)
        if ov != self.override:
            self.override = ov
            self.send_ur(cmd="speed", value=ov[0])
            self.send_scara(cmd="speed", value=ov[1])
        # stack light: red = fault, amber = warning / stopped / cycle stop, green = automatic
        blink = int(self.getTime() * 2) % 2 == 0
        state = {"RED": self.fault is not None and blink,
                 "AMBER": self.fault is None and (self.scan_warn or self.scan_stop or self.cycle_stop),
                 "GREEN": self.fault is None and not self.scan_stop}
        for k, on in state.items():
            self.lamp[k].setSFColor([float(v) for v in LAMP[k].split()] if on else [0, 0, 0])

    def reset(self):
        if self.fault is None:
            return
        if self.estop:
            self.log("Reset refused: e-stop still pressed")
        elif self.door_open:
            self.log("Reset refused: door still open")
        else:
            self.log("Reset: %s cleared" % self.fault)
            self.fault = None

    def command(self, c):
        c = c.upper()
        if c == "E":
            self.estop = not self.estop
            if not self.estop:
                self.log("E-stop released (reset required)")
        elif c == "D":
            self.door_open = not self.door_open
            if not self.door_open:
                self.log("Guard door closed (reset required)")
        elif c == "R":
            self.reset()
        elif c == "P":
            self.tx_op.send("walk")
            self.log("Operator walks to the cart opening")
        elif c == "S":
            self.cycle_stop = not self.cycle_stop
            self.log("End-of-cycle stop" if self.cycle_stop else "Cycle restarted")

    # -------------------------------------------------------------- robots
    def on_scara(self, m):
        e = m["evt"]
        if e == "ready":
            self.scara["ready"] = True
            self.fk_check["scara"] = m["fk_err"]
            self.log("SCARA ready (FK vs simulator %.2f mm)" % (m["fk_err"] * 1000))
        elif e == "picked":
            self.belt = [it for it in self.belt if it[0] != self.scara["part"]]
        elif e == "placed":
            self.station["parts"].append(m["part"])
            self.stats["parts"] += 1
        elif e == "done":
            self.scara["busy"] = False
            self.scara["part"] = None
            self.scara["gripped"] = False
        elif e == "error":
            self.robot_error("SCARA", m["msg"])

    def on_ur(self, m):
        e = m["evt"]
        if e == "ready":
            self.ur["ready"] = True
            self.fk_check["ur"] = m["fk_err"]
            self.log("UR5e ready (FK vs simulator %.2f mm)" % (m["fk_err"] * 1000))
        elif e == "station_free":
            self.log("Full tray lifted from the station")
        elif e == "cart_loaded":
            self.cart_load.append(self.ur["job"]["full"])
            self.stats["trays"] += 1
            self.log("Tray %d/%d on the cart" % (len(self.cart_load), L.CART_SLOTS))
        elif e == "station_loaded":
            self.station = {"tray": self.ur["job"]["empty"], "parts": []}
        elif e == "done":
            self.ur["busy"] = False
            if len(self.cart_load) >= L.CART_SLOTS:
                self.cart["state"] = "leaving"
                self.log("Cart full: leaving through the opening")
        elif e == "error":
            self.robot_error("UR5e", m["msg"])

    def robot_error(self, who, msg):
        self.stats["errors"].append("%s: %s" % (who, msg))
        self.log("%s error: %s" % (who, msg))
        if self.fault is None:
            self.fault = "%s FAULT" % who

    def dispatch(self):
        if not self.cell_ok() or self.cycle_stop:
            return
        sc, ur = self.scara, self.ur
        n_parts = len(self.station["parts"])
        tray_here = self.sees("B2")
        # SCARA: fill the tray at the station
        if (sc["ready"] and not sc["busy"] and not ur["busy"] and tray_here and n_parts < L.PARTS_PER_TRAY
                and self.sees("B1") and self.belt and self.belt[0][1] >= L.PICK[0] - 1e-6):
            part = self.belt[0][0]
            sc.update(busy=True, part=part, gripped=True, id=sc["id"] + 1)
            self.send_scara(cmd="pick_place", id=sc["id"], part=part, pick=list(L.PICK),
                            place=list(L.pocket_positions()[n_parts]))
        # UR5e: swap the full tray as soon as the SCARA is clear
        if (ur["ready"] and not ur["busy"] and not sc["busy"] and n_parts >= L.PARTS_PER_TRAY and tray_here
                and self.cart["state"] == "docked" and self.sees("B7") and len(self.cart_load) < L.CART_SLOTS
                and self.magazine):
            k = len(self.cart_load)
            empty = self.magazine.pop()
            job = {"full": (self.station["tray"], list(self.station["parts"])), "empty": empty}
            ur.update(busy=True, id=ur["id"] + 1, job=job)
            mag_k = L.MAG_COUNT - 1 - len(self.magazine)
            self.send_ur(cmd="swap", id=ur["id"], full=[job["full"][0]] + job["full"][1],
                         station=list(L.station_grip()), cart=list(L.cart_grip(k)),
                         mag=list(L.magazine_grip(mag_k)), empty=empty)
            self.log("UR5e: tray swap %d" % ur["id"])

    # -------------------------------------------------------------- operator view
    def overlay(self):
        head = "FAULT: %s (R = reset)" % self.fault if self.fault else (
            "PROTECTIVE STOP" if self.scan_stop else "REDUCED SPEED" if self.scan_warn else "AUTOMATIC")
        self.setLabel(0, "UR5e + Epson SCARA T6 kitting cell  |  %s" % head, 0.01, 0.01, 0.07,
                      0xFF5050 if self.fault else 0xFFD040 if (self.scan_stop or self.scan_warn) else 0xFFFFFF, 0)
        self.setLabel(1, "parts %d   trays %d   carts %d   |   %s" % (
            self.stats["parts"], self.stats["trays"], self.stats["carts"], self.events[-1][8:] if self.events else ""),
            0.01, 0.05, 0.06, 0xFFE070, 0)
        self.setLabel(2, "keys: E e-stop  D door  R reset  P operator  S cycle stop", 0.01, 0.95, 0.05, 0xCCCCCC, 0)

    # -------------------------------------------------------------- self-test
    def smoke_script(self, t):
        """Scripted events during --smoke runs."""
        plan = getattr(self, "_plan", None)
        if plan is None:
            T = self.smoke
            # e-stop, reset refused while pressed, release + reset, then the operator walks to the opening
            self._plan = plan = [(T * 0.55, "E"), (T * 0.55 + 3, "R"), (T * 0.55 + 4, "E"), (T * 0.55 + 5, "R"),
                                 (T * 0.65, "P")]
        while plan and t >= plan[0][0]:
            self.command(plan.pop(0)[1])

    def smoke_verdict(self):
        ok, why = True, []
        need = {"carts": 1, "trays": L.CART_SLOTS}
        for k, v in need.items():
            if self.stats[k] < v:
                ok = False
                why.append("%s=%d < %d" % (k, self.stats[k], v))
        if self.stats["errors"]:
            ok = False
            why += self.stats["errors"]
        for name, err in self.fk_check.items():
            print("[SMOKE] FK %s vs simulator: %.2f mm" % (name, err * 1000))
            if err > 0.002:
                ok = False
                why.append("FK mismatch %s %.1f mm" % (name, err * 1000))
        if len(self.fk_check) < 2:
            ok, why = False, why + ["robot never ready"]
        for flag, label in [(getattr(self, "_saw_estop_hold", False), "e-stop did not hold the robots"),
                            (getattr(self, "_saw_refused", False), "reset with e-stop pressed was not refused"),
                            (getattr(self, "_saw_scan_stop", False), "scanner stop never triggered"),
                            (getattr(self, "_saw_scan_resume", False), "no automatic restart after the scanner")]:
            if not flag:
                ok = False
                why.append(label)
        print("[SMOKE] stats %s" % json.dumps(self.stats))
        print("[SMOKE] %s %s" % ("PASS" if ok else "FAIL", "; ".join(why)))
        return ok

    # -------------------------------------------------------------- main loop
    def run(self):
        dt = self.dt / 1000.0
        self._wall0 = time.time()
        self.log("Cell started")
        while self.step(self.dt) != -1:
            t = self.getTime()
            k = self.kb.getKey()
            while k > 0:
                if 0 < k < 128:
                    self.command(chr(k))
                k = self.kb.getKey()
            cd = self.me.getField("customData")
            if cd.getSFString():
                for c in cd.getSFString():
                    self.command(c)
                cd.setSFString("")
            if self.smoke:
                self.smoke_script(t)
            while self.rx_s.getQueueLength() > 0:
                self.on_scara(json.loads(self.rx_s.getString()))
                self.rx_s.nextPacket()
            while self.rx_u.getQueueLength() > 0:
                self.on_ur(json.loads(self.rx_u.getString()))
                self.rx_u.nextPacket()
            self.safety()
            if self.smoke:
                if self.estop and self.held:
                    self._saw_estop_hold = True
                if self.estop and self.events and "refused" in self.events[-1]:
                    self._saw_refused = True
                if self.scan_stop:
                    self._saw_scan_stop = True
                if getattr(self, "_saw_scan_stop", False) and not self.scan_stop and not self.held:
                    self._saw_scan_resume = True
            self.dispatch()
            self.run_belt(dt)
            self.run_cart(dt)
            self.run_door(dt)
            self.overlay()
            if self.smoke and int(t / dt) % int(10 / dt) == 0:
                print("[SMOKE] t=%.0f s  wall=%.0f s  %s  B1 %d B5 %d B2 %d B7 %d  scara %s ur %s  belt %d  station %s" % (
                    t, time.time() - self._wall0, json.dumps(self.stats),
                    *(int(self.sees(n)) for n in ("B1", "B5", "B2", "B7")),
                    {k: v for k, v in self.scara.items() if k in ("ready", "busy")},
                    {k: v for k, v in self.ur.items() if k in ("ready", "busy")}, len(self.belt),
                    (self.station["tray"], len(self.station["parts"]),
                     [round(v, 3) for v in self.node[self.station["tray"]].getPosition()],
                     [round(v, 3) for v in self.node[self.station["tray"]].getOrientation()])))
            if self.smoke and t >= self.smoke:
                ok = self.smoke_verdict()
                for _ in range(10):         # let Webots collect the controller output before quitting
                    self.step(self.dt)
                self.simulationQuit(0 if ok else 1)
                self.step(self.dt)
                break


PLC().run()

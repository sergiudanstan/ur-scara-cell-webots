"""Cell layout shared by the world generator, the PLC and both robot controllers (world frame, metres, z up).

Plan view (x to the right, y up):

      magazine of empty trays (MAG)           UR5e on the table (UR_BASE)
             SCARA T6 (SCARA_BASE)
   infeed conveyor ----> PICK      tray station (STATION)
                                                   outfeed cart (CART, 2 x 2 x 2 trays) -> leaves through the gate
"""
import math

# ------------------------------------------------------------------ table and robots
TABLE_X0, TABLE_X1, TABLE_Y0, TABLE_Y1, TABLE_Z = -0.15, 1.10, -0.30, 0.70, 0.75
SCARA_RISER = 0.05
SCARA_BASE = (0.0, 0.32, TABLE_Z + SCARA_RISER, -math.pi / 2)   # x, y, z, yaw (arm points to -y at q = 0)
SCARA_TOOL = 0.06               # hand slot -> suction cup lip
SCARA_CLEAR_Z = 0.85            # transfer height of the TCP
UR_BASE = (0.85, -0.05, TABLE_Z, math.pi)
UR_TOOL = 0.165                 # tool slot -> finger tip centre (TCP)
UR_CLEAR_Z = 1.02
UR_Q_HOME = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]   # seed only; the real home is UR_HOME_TCP
UR_HOME_TCP = (0.55, 0.15, 1.15, 0.0)

# ------------------------------------------------------------------ parts and trays
PART_R, PART_H = 0.022, 0.035
TRAY_L, TRAY_W, TRAY_H, TRAY_FLOOR = 0.24, 0.16, 0.05, 0.006
POCKETS = [(-0.075, -0.04), (0.0, -0.04), (0.075, -0.04), (-0.075, 0.04), (0.0, 0.04), (0.075, 0.04)]
PARTS_PER_TRAY = len(POCKETS)

# ------------------------------------------------------------------ infeed conveyor (belt along +x, ends at the stop)
CONV_Y = -0.16
CONV_X0, CONV_X1 = -1.60, -0.20
CONV_Z = 0.76                   # belt top
CONV_W = 0.12
BELT_SPEED = 0.30               # m/s
PART_GAP = 0.052                # minimum pitch between parts in accumulation
PICK = (CONV_X1 - 0.045, CONV_Y, CONV_Z + PART_H)      # TCP on top of the part waiting at the stop
SPAWN_X = CONV_X0 + 0.08
ACCU_X = PICK[0] - 4 * PART_GAP                       # accumulation-full sensor position
FEED_PERIOD = 1.2               # s between two parts at the upstream machine

# ------------------------------------------------------------------ stations
STATION = (0.32, -0.10)         # tray centre at the SCARA place station (long side along x)
MAG = (0.55, 0.50)              # magazine of empty trays (stacked)
MAG_COUNT = 8
CART = (0.85, -0.60)            # cart centre in the loading position
CART_TOP = 0.45
CART_COLS = [(-0.14, 0.105), (0.14, 0.105), (-0.14, -0.105), (0.14, -0.105)]
CART_LAYERS = 2
CART_SLOTS = len(CART_COLS) * CART_LAYERS
CART_OUT = (0.85, -2.6)         # cart position when it has left the cell


def pocket_positions():
    """TCP targets (world) to place a part in each pocket of the tray at the station."""
    z = TABLE_Z + TRAY_FLOOR + PART_H + 0.002
    return [(STATION[0] + dx, STATION[1] + dy, z) for dx, dy in POCKETS]


# UR grips the tray on its two short walls, closing along x; TCP GRIP_H above the tray bottom (tray origin)
GRIP_H = TRAY_H * 0.6


def station_grip():
    return (STATION[0], STATION[1], TABLE_Z + GRIP_H, 0.0)


def magazine_grip(k):
    """k-th tray from the top (k = 0 is the top tray of a full magazine)."""
    level = MAG_COUNT - 1 - k
    return (MAG[0], MAG[1], TABLE_Z + level * TRAY_H + GRIP_H, 0.0)


def cart_grip(k):
    col = CART_COLS[k % len(CART_COLS)]
    layer = k // len(CART_COLS)
    return (CART[0] + col[0], CART[1] + col[1], CART_TOP + layer * TRAY_H + GRIP_H, 0.0)


# ------------------------------------------------------------------ safety
FENCE = (-1.05, 1.45, -0.55, 1.10)      # x0, x1, y0, y1 of the guard around the SCARA side
DOOR = (-1.05, 0.10, 0.75)              # door in the west wall: x, y centre, width
SCANNER = (1.30, -0.55)                 # area scanner in front of the cart opening
WARN_R, STOP_R = 1.6, 0.9               # UR speed-and-separation zones (radius from the scanner)
UR_SLOW = 0.25
GATE = (0.30, 1.30)                     # x-range of the cart opening in the south side (scanner-guarded)
OPERATOR_HOME = (2.3, 0.2)             # beside the console, facing the cell
OPERATOR_PATH = [(2.3, 0.2), (2.0, -1.2), (1.15, -0.95), (2.0, -1.2), (2.3, 0.2)]

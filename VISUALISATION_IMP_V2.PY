import sys
import threading
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.gridspec as gridspec
from collections import deque
import time

# ── CONFIGURATION ─────────────────────────────────────────────

BAUD_RATE = 115200          # Must match ESP32 serial baud rate
TRAIL_LEN = 80              # Number of past positions shown
AUTO_MARGIN = 1.0           # Padding around auto-scaled plot
REFRESH_MS = 100            # Plot refresh interval

# Known beacon positions (for visualization only)
BEACON_POSITIONS = {
    "BCN_A": (0.0, 0.0),
    "BCN_B": (3.0, 0.0),
    "BCN_C": (0.0, 3.0),
}

# ── SHARED STATE (THREAD-SAFE) ───────────────────────────────

state = {
    "x": 0.0,               # Current X position
    "y": 0.0,               # Current Y position
    "mask": 0,              # Beacon availability bitmask
    "tag": "??",            # Tag ID
    "last_update": None,    # Timestamp of last data
    "lock": threading.Lock()
}

# Store movement history
trail = deque(maxlen=TRAIL_LEN)

# Store all received positions (for auto-scaling)
all_x = deque(maxlen=500)
all_y = deque(maxlen=500)

# ── AUTO-DETECT SERIAL PORT ─────────────────────────────────

def find_port():
    keywords = ["CH340", "CP210", "UART", "USB Serial", "ESP"]
    ports = serial.tools.list_ports.comports()

    for p in ports:
        desc = (p.description or "") + (p.manufacturer or "")
        if any(k.lower() in desc.lower() for k in keywords):
            return p.device

    return ports[0].device if ports else None


# ── SERIAL READER THREAD ────────────────────────────────────

def serial_reader(port, baud):

    # Open serial connection
    ser = serial.Serial(port, baud, timeout=1)

    while True:
        try:
            line = ser.readline().decode().strip()

            # Only process valid data lines
            if not line.startswith("DATA,"):
                continue

            parts = line.split(",")

            # Expected format: DATA,Tag,X,Y,Mask
            if len(parts) < 5:
                continue

            _, tag, sx, sy, smask = parts
            x, y, mask = float(sx), float(sy), int(smask)

            # Skip invalid positions
            if x == -1 and y == -1:
                with state["lock"]:
                    state["mask"] = mask
                continue

            # Update shared state safely
            with state["lock"]:
                state["x"] = x
                state["y"] = y
                state["mask"] = mask
                state["tag"] = tag
                state["last_update"] = time.time()

                # Store history
                trail.append((x, y))
                all_x.append(x)
                all_y.append(y)

        except:
            continue


# ── BEACON STATUS DECODER ───────────────────────────────────

def mask_to_status(mask):
    return {
        "BCN_A": bool(mask & 1),
        "BCN_B": bool(mask & 2),
        "BCN_C": bool(mask & 4),
    }


# ── BUILD PLOT LAYOUT ───────────────────────────────────────

def build_plot():

    fig = plt.figure(figsize=(11, 6))
    gs = gridspec.GridSpec(1, 2, width_ratios=[3, 1])

    ax_map = fig.add_subplot(gs[0])      # Left → map
    ax_status = fig.add_subplot(gs[1])   # Right → status panel

    return fig, ax_map, ax_status


# ── UPDATE PLOT (CALLED PERIODICALLY) ───────────────────────

def update_plot(fig, ax_map, ax_status):

    # Copy shared data safely
    with state["lock"]:
        x = state["x"]
        y = state["y"]
        mask = state["mask"]
        tag = state["tag"]
        last = state["last_update"]
        trail_snap = list(trail)
        xs = list(all_x)
        ys = list(all_y)

    # Clear map
    ax_map.cla()

    # ── AUTO-SCALING ──
    if xs and ys:
        xmin = min(xs) - AUTO_MARGIN
        xmax = max(xs) + AUTO_MARGIN
        ymin = min(ys) - AUTO_MARGIN
        ymax = max(ys) + AUTO_MARGIN

        # Ensure minimum visible area
        if xmax - xmin < 3:
            xmin -= 1; xmax += 1
        if ymax - ymin < 3:
            ymin -= 1; ymax += 1

        ax_map.set_xlim(xmin, xmax)
        ax_map.set_ylim(ymin, ymax)

    ax_map.grid(True)

    # ── DRAW BEACONS ──
    status = mask_to_status(mask)

    for name, (bx, by) in BEACON_POSITIONS.items():
        color = "green" if status[name] else "red"
        ax_map.plot(bx, by, "^", color=color)
        ax_map.text(bx, by, name)

    # ── DRAW TRAJECTORY ──
    if len(trail_snap) > 1:
        tx = [p[0] for p in trail_snap]
        ty = [p[1] for p in trail_snap]
        ax_map.plot(tx, ty, color="blue")

    # ── DRAW CURRENT POSITION ──
    if last:
        stale = time.time() - last > 3

        # Grey if data is outdated
        color = "grey" if stale else "yellow"

        ax_map.plot(x, y, "o", color=color)
        ax_map.text(x, y, f"{tag} ({x:.2f},{y:.2f})")

    fig.canvas.draw_idle()


# ── MAIN FUNCTION ───────────────────────────────────────────

def main():

    port = sys.argv[1] if len(sys.argv) > 1 else find_port()

    if not port:
        print("No serial port found")
        return

    # Start background thread
    threading.Thread(target=serial_reader, args=(port, BAUD_RATE), daemon=True).start()

    fig, ax_map, ax_status = build_plot()

    # Timer for continuous updates
    timer = fig.canvas.new_timer(interval=REFRESH_MS)
    timer.add_callback(update_plot, fig, ax_map, ax_status)
    timer.start()

    plt.show()


if __name__ == "__main__":
    main()

"""
BLE Tag Live Visualizer
-----------------------
Reads DATA,<tagID>,<x>,<y>,<mask> lines from the gateway ESP32 over serial.
Displays:
  - Live 2D map with tag position (auto-scaling)
  - Beacon mask status (BCN_A, BCN_B, BCN_C)
  - Position history trail

Requirements:
    pip install pyserial matplotlib

Usage:
    python tag_visualizer.py              # auto-detects port
    python tag_visualizer.py COM3         # Windows
    python tag_visualizer.py /dev/ttyUSB0 # Linux/Mac
"""

import sys
import threading
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.gridspec as gridspec
from collections import deque
import time

# ── CONFIG ─────────────────────────────────────────────────────────────────
BAUD_RATE    = 115200
TRAIL_LEN    = 80        # how many past positions to draw
AUTO_MARGIN  = 1.0       # metres of padding around auto-scale bounds
REFRESH_MS   = 100       # plot refresh interval in ms

# Beacon positions for drawing on map (set to your real coords, or None to hide)
BEACON_POSITIONS = {
    "BCN_A": (0.0, 0.0),
    "BCN_B": (3.0, 0.0),
    "BCN_C": (0.0, 3.0),
}
# ───────────────────────────────────────────────────────────────────────────

# Shared state (written by serial thread, read by plot thread)
state = {
    "x": 0.0,
    "y": 0.0,
    "mask": 0,
    "tag": "??",
    "last_update": None,
    "lock": threading.Lock(),
}
trail = deque(maxlen=TRAIL_LEN)
all_x = deque(maxlen=500)
all_y = deque(maxlen=500)

def find_port():
    """Auto-detect the first ESP32/CH340/CP210x serial port."""
    keywords = ["CH340", "CP210", "UART", "USB Serial", "ESP"]
    ports = serial.tools.list_ports.comports()
    for p in ports:
        desc = (p.description or "") + (p.manufacturer or "")
        if any(k.lower() in desc.lower() for k in keywords):
            print(f"Auto-detected port: {p.device} ({p.description})")
            return p.device
    if ports:
        print(f"No ESP32 found, using first port: {ports[0].device}")
        return ports[0].device
    return None

def serial_reader(port, baud):
    """Background thread: reads serial and updates shared state."""
    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"Opened {port} @ {baud}")
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        return

    while True:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line.startswith("DATA,"):
                continue
            parts = line.split(",")
            if len(parts) < 5:
                continue
            _, tag, sx, sy, smask = parts[0], parts[1], parts[2], parts[3], parts[4]
            x, y, mask = float(sx), float(sy), int(smask)

            # skip sentinel invalid positions
            if x == -1 and y == -1:
                with state["lock"]:
                    state["mask"] = mask
                continue

            with state["lock"]:
                state["x"] = x
                state["y"] = y
                state["mask"] = mask
                state["tag"] = tag
                state["last_update"] = time.time()
                trail.append((x, y))
                all_x.append(x)
                all_y.append(y)

        except (ValueError, serial.SerialException):
            continue

def mask_to_status(mask):
    """Return per-beacon active status from bitmask."""
    return {
        "BCN_A": bool(mask & 1),
        "BCN_B": bool(mask & 2),
        "BCN_C": bool(mask & 4),
    }

def build_plot():
    fig = plt.figure(figsize=(11, 6), facecolor="#1a1a2e")
    fig.canvas.manager.set_window_title("BLE Tag Visualizer")
    gs = gridspec.GridSpec(1, 2, width_ratios=[3, 1], wspace=0.35)

    ax_map    = fig.add_subplot(gs[0])
    ax_status = fig.add_subplot(gs[1])

    for ax in (ax_map, ax_status):
        ax.set_facecolor("#0f0f1e")
        for spine in ax.spines.values():
            spine.set_edgecolor("#444466")

    return fig, ax_map, ax_status

def update_plot(fig, ax_map, ax_status):
    with state["lock"]:
        x      = state["x"]
        y      = state["y"]
        mask   = state["mask"]
        tag    = state["tag"]
        last   = state["last_update"]
        trail_snap = list(trail)
        ax_vals = list(all_x), list(all_y)

    # ── MAP ──────────────────────────────────────────────────────────────
    ax_map.cla()
    ax_map.set_facecolor("#0f0f1e")
    ax_map.set_title("Live Position Map", color="white", fontsize=12, pad=8)
    ax_map.tick_params(colors="#888899")
    ax_map.xaxis.label.set_color("#888899")
    ax_map.yaxis.label.set_color("#888899")
    ax_map.set_xlabel("X (m)"); ax_map.set_ylabel("Y (m)")
    for spine in ax_map.spines.values():
        spine.set_edgecolor("#444466")

    # auto-scale
    xs, ys = ax_vals
    if xs and ys:
        xmin = min(xs) - AUTO_MARGIN;  xmax = max(xs) + AUTO_MARGIN
        ymin = min(ys) - AUTO_MARGIN;  ymax = max(ys) + AUTO_MARGIN
        # keep a minimum window size
        if xmax - xmin < 3: xmin -= 1; xmax += 1
        if ymax - ymin < 3: ymin -= 1; ymax += 1
        ax_map.set_xlim(xmin, xmax)
        ax_map.set_ylim(ymin, ymax)

    ax_map.grid(True, color="#2a2a4a", linewidth=0.5)

    # draw beacons
    status = mask_to_status(mask)
    for bname, (bx, by) in BEACON_POSITIONS.items():
        active = status.get(bname, False)
        color  = "#00ff99" if active else "#ff4466"
        ax_map.plot(bx, by, "^", markersize=13, color=color,
                    markeredgecolor="white", markeredgewidth=0.8, zorder=5)
        ax_map.annotate(bname, (bx, by),
                        textcoords="offset points", xytext=(6, 6),
                        color=color, fontsize=8, fontweight="bold")

    # draw trail
    if len(trail_snap) > 1:
        tx = [p[0] for p in trail_snap]
        ty = [p[1] for p in trail_snap]
        # fade: alpha increases toward newest
        for i in range(1, len(tx)):
            alpha = 0.15 + 0.75 * (i / len(tx))
            ax_map.plot(tx[i-1:i+1], ty[i-1:i+1],
                        color="#4488ff", linewidth=1.2, alpha=alpha, zorder=3)

    # draw tag dot
    if last is not None:
        stale = time.time() - last > 3.0
        dot_color = "#888888" if stale else "#ffdd00"
        ax_map.plot(x, y, "o", markersize=14, color=dot_color,
                    markeredgecolor="white", markeredgewidth=1.5, zorder=10)
        ax_map.annotate(f"  {tag}\n  ({x:.2f}, {y:.2f})",
                        (x, y), color="white", fontsize=9,
                        fontweight="bold", zorder=11)

    # ── BEACON STATUS PANEL ───────────────────────────────────────────────
    ax_status.cla()
    ax_status.set_facecolor("#0f0f1e")
    ax_status.set_title("Beacon Status", color="white", fontsize=12, pad=8)
    ax_status.axis("off")

    for spine in ax_status.spines.values():
        spine.set_edgecolor("#444466")

    beacons = ["BCN_A", "BCN_B", "BCN_C"]
    for i, bname in enumerate(beacons):
        active = status.get(bname, False)
        ypos   = 0.75 - i * 0.28
        color  = "#00ff99" if active else "#ff4466"
        label  = "● ONLINE" if active else "● OFFLINE"

        # card background
        rect = mpatches.FancyBboxPatch(
            (0.05, ypos - 0.07), 0.9, 0.18,
            boxstyle="round,pad=0.02",
            linewidth=1.2,
            edgecolor=color,
            facecolor="#1a1a2e",
            transform=ax_status.transAxes,
            zorder=2
        )
        ax_status.add_patch(rect)

        ax_status.text(0.5, ypos + 0.05, bname,
                       transform=ax_status.transAxes,
                       ha="center", color="white",
                       fontsize=11, fontweight="bold")
        ax_status.text(0.5, ypos - 0.02, label,
                       transform=ax_status.transAxes,
                       ha="center", color=color, fontsize=9)

    # mask raw value
    ax_status.text(0.5, 0.08, f"mask = {mask:03b}  ({mask})",
                   transform=ax_status.transAxes,
                   ha="center", color="#888899", fontsize=8)

    # last update timestamp
    if last:
        age = time.time() - last
        ts_str = f"Last RX: {age:.1f}s ago"
        ts_col = "#ffdd00" if age < 3 else "#ff4466"
    else:
        ts_str = "No data yet"
        ts_col = "#888899"

    ax_status.text(0.5, 0.01, ts_str,
                   transform=ax_status.transAxes,
                   ha="center", color=ts_col, fontsize=8)

    fig.canvas.draw_idle()

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    if not port:
        print("No serial port found. Plug in the gateway or pass port as argument.")
        print("  Example: python tag_visualizer.py COM3")
        sys.exit(1)

    t = threading.Thread(target=serial_reader, args=(port, BAUD_RATE), daemon=True)
    t.start()

    fig, ax_map, ax_status = build_plot()

    def on_timer(interval):
        update_plot(fig, ax_map, ax_status)
        fig.canvas.get_tk_widget().after(interval, on_timer, interval) \
            if hasattr(fig.canvas, 'get_tk_widget') else None

    # use matplotlib's animation timer for cross-platform refresh
    timer = fig.canvas.new_timer(interval=REFRESH_MS)
    timer.add_callback(update_plot, fig, ax_map, ax_status)
    timer.start()

    plt.tight_layout(pad=1.5)
    plt.show()

if __name__ == "__main__":
    main()
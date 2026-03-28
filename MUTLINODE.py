"""
BLE Tag Live Visualizer (Multi-Tag)
-------------------------------------
Reads DATA,<tagID>,<x>,<y>,<mask> lines from the gateway ESP32 over serial.
Displays:
  - Live 2D map with ALL tags shown simultaneously
  - Per-tag color-coded dots and trails
  - Beacon mask status panel
  - Auto-scaling map

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

# ── CONFIG ──────────────────────────────────────────────────────────────────
BAUD_RATE   = 115200
TRAIL_LEN   = 80       # past positions per tag
AUTO_MARGIN = 1.0      # metres padding on auto-scale
REFRESH_MS  = 100      # plot refresh interval

BEACON_POSITIONS = {
    "BCN_A": (0.0, 0.0),
    "BCN_B": (3.0, 0.0),
    "BCN_C": (0.0, 3.0),
}

# One color per tag — add more if you have more tags
TAG_COLORS = [
    "#ffdd00",  # T1 — yellow
    "#00cfff",  # T2 — cyan
    "#ff6ec7",  # T3 — pink
    "#aaff44",  # T4 — lime
    "#ff8c42",  # T5 — orange
]
# ────────────────────────────────────────────────────────────────────────────

# Per-tag state — keyed by tagID string
tags = {}
all_x = deque(maxlen=1000)
all_y = deque(maxlen=1000)
data_lock = threading.Lock()
latest_mask = 0


def find_port():
    keywords = ["CH340", "CP210", "UART", "USB Serial", "ESP"]
    ports = serial.tools.list_ports.comports()
    for p in ports:
        desc = (p.description or "") + (p.manufacturer or "")
        if any(k.lower() in desc.lower() for k in keywords):
            print(f"Auto-detected: {p.device} ({p.description})")
            return p.device
    if ports:
        print(f"Using first port: {ports[0].device}")
        return ports[0].device
    return None


def get_or_create_tag(tag_id):
    """Return existing tag state or create a new one."""
    if tag_id not in tags:
        color_idx = len(tags) % len(TAG_COLORS)
        tags[tag_id] = {
            "x": 0.0,
            "y": 0.0,
            "mask": 0,
            "last_update": None,
            "trail": deque(maxlen=TRAIL_LEN),
            "color": TAG_COLORS[color_idx],
        }
        print(f"New tag registered: {tag_id} → color {TAG_COLORS[color_idx]}")
    return tags[tag_id]


def serial_reader(port, baud):
    global latest_mask
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
            tag_id = parts[1]
            x, y, mask = float(parts[2]), float(parts[3]), int(parts[4])

            with data_lock:
                latest_mask = mask
                t = get_or_create_tag(tag_id)
                t["mask"] = mask

                if x == -1 and y == -1:
                    continue  # beacon missing — mask updated only

                t["x"] = x
                t["y"] = y
                t["last_update"] = time.time()
                t["trail"].append((x, y))
                all_x.append(x)
                all_y.append(y)

        except (ValueError, serial.SerialException):
            continue


def mask_to_status(mask):
    return {
        "BCN_A": bool(mask & 1),
        "BCN_B": bool(mask & 2),
        "BCN_C": bool(mask & 4),
    }


def build_plot():
    fig = plt.figure(figsize=(12, 6), facecolor="#1a1a2e")
    fig.canvas.manager.set_window_title("BLE Tag Visualizer — Multi-Tag")
    gs = gridspec.GridSpec(1, 2, width_ratios=[3, 1], wspace=0.35)
    ax_map    = fig.add_subplot(gs[0])
    ax_status = fig.add_subplot(gs[1])
    for ax in (ax_map, ax_status):
        ax.set_facecolor("#0f0f1e")
        for spine in ax.spines.values():
            spine.set_edgecolor("#444466")
    return fig, ax_map, ax_status


def update_plot(fig, ax_map, ax_status):
    with data_lock:
        tags_snap = {tid: dict(t, trail=list(t["trail"])) for tid, t in tags.items()}
        xs        = list(all_x)
        ys        = list(all_y)
        mask      = latest_mask

    # ── MAP ────────────────────────────────────────────────────────────────
    ax_map.cla()
    ax_map.set_facecolor("#0f0f1e")
    ax_map.set_title("Live Position Map", color="white", fontsize=12, pad=8)
    ax_map.tick_params(colors="#888899")
    ax_map.xaxis.label.set_color("#888899")
    ax_map.yaxis.label.set_color("#888899")
    ax_map.set_xlabel("X (m)")
    ax_map.set_ylabel("Y (m)")
    for spine in ax_map.spines.values():
        spine.set_edgecolor("#444466")

    # auto-scale bounds: include beacons + recent tag positions + margin
    plot_x = [p[0] for p in BEACON_POSITIONS.values()]
    plot_y = [p[1] for p in BEACON_POSITIONS.values()]

    for t in tags_snap.values():
        if t["last_update"] is None:
            continue
        if t["trail"]:
            plot_x.extend(p[0] for p in t["trail"])
            plot_y.extend(p[1] for p in t["trail"])
        else:
            plot_x.append(t["x"])
            plot_y.append(t["y"])

    x_min, x_max = min(plot_x), max(plot_x)
    y_min, y_max = min(plot_y), max(plot_y)

    # keep a minimum visible range to avoid jitter when points are close
    min_span = 1.0
    if (x_max - x_min) < min_span:
        cx = (x_min + x_max) / 2.0
        x_min = cx - min_span / 2.0
        x_max = cx + min_span / 2.0
    if (y_max - y_min) < min_span:
        cy = (y_min + y_max) / 2.0
        y_min = cy - min_span / 2.0
        y_max = cy + min_span / 2.0

    ax_map.set_xlim(x_min - AUTO_MARGIN, x_max + AUTO_MARGIN)
    ax_map.set_ylim(y_min - AUTO_MARGIN, y_max + AUTO_MARGIN)
    ax_map.set_aspect("equal", adjustable="box")

    ax_map.grid(True, color="#2a2a4a", linewidth=0.5)

    # draw beacons
    bcn_status = mask_to_status(mask)
    for bname, (bx, by) in BEACON_POSITIONS.items():
        active = bcn_status.get(bname, False)
        color  = "#00ff99" if active else "#ff4466"
        ax_map.plot(bx, by, "^", markersize=13, color=color,
                    markeredgecolor="white", markeredgewidth=0.8, zorder=5)
        ax_map.annotate(bname, (bx, by),
                        textcoords="offset points", xytext=(6, 6),
                        color=color, fontsize=8, fontweight="bold")

    # draw each tag's trail + dot
    for tid, t in tags_snap.items():
        color = t["color"]
        trail = t["trail"]

        if len(trail) > 1:
            tx = [p[0] for p in trail]
            ty = [p[1] for p in trail]
            for i in range(1, len(tx)):
                alpha = 0.1 + 0.8 * (i / len(tx))
                ax_map.plot(tx[i-1:i+1], ty[i-1:i+1],
                            color=color, linewidth=1.2, alpha=alpha, zorder=3)

        if t["last_update"] is not None:
            stale = time.time() - t["last_update"] > 3.0
            dot_color = "#555566" if stale else color
            ax_map.plot(t["x"], t["y"], "o", markersize=14,
                        color=dot_color,
                        markeredgecolor="white", markeredgewidth=1.5, zorder=10)
            ax_map.annotate(f"  {tid}\n  ({t['x']:.2f}, {t['y']:.2f})",
                            (t["x"], t["y"]),
                            color=color, fontsize=9, fontweight="bold", zorder=11)

    # tag legend
    if tags_snap:
        handles = [mpatches.Patch(color=t["color"], label=tid)
                   for tid, t in tags_snap.items()]
        ax_map.legend(handles=handles, loc="upper right",
                      facecolor="#1a1a2e", edgecolor="#444466",
                      labelcolor="white", fontsize=9)

    # ── BEACON STATUS PANEL ────────────────────────────────────────────────
    ax_status.cla()
    ax_status.set_facecolor("#0f0f1e")
    ax_status.set_title("Beacon Status", color="white", fontsize=12, pad=8)
    ax_status.axis("off")
    for spine in ax_status.spines.values():
        spine.set_edgecolor("#444466")

    beacons = ["BCN_A", "BCN_B", "BCN_C"]
    for i, bname in enumerate(beacons):
        active = bcn_status.get(bname, False)
        ypos   = 0.75 - i * 0.25
        color  = "#00ff99" if active else "#ff4466"
        label  = "● ONLINE" if active else "● OFFLINE"

        rect = mpatches.FancyBboxPatch(
            (0.05, ypos - 0.07), 0.9, 0.17,
            boxstyle="round,pad=0.02",
            linewidth=1.2, edgecolor=color,
            facecolor="#1a1a2e",
            transform=ax_status.transAxes, zorder=2
        )
        ax_status.add_patch(rect)
        ax_status.text(0.5, ypos + 0.04, bname,
                       transform=ax_status.transAxes,
                       ha="center", color="white", fontsize=11, fontweight="bold")
        ax_status.text(0.5, ypos - 0.02, label,
                       transform=ax_status.transAxes,
                       ha="center", color=color, fontsize=9)

    # active tag count
    active_tags = sum(1 for t in tags_snap.values()
                      if t["last_update"] and time.time() - t["last_update"] < 3.0)
    ax_status.text(0.5, 0.10,
                   f"Tags active: {active_tags} / {len(tags_snap)}",
                   transform=ax_status.transAxes,
                   ha="center", color="#888899", fontsize=8)

    ax_status.text(0.5, 0.04, f"mask = {mask:03b}  ({mask})",
                   transform=ax_status.transAxes,
                   ha="center", color="#555566", fontsize=7)

    fig.canvas.draw_idle()


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    if not port:
        print("No serial port found.")
        print("  Usage: python tag_visualizer.py COM3")
        sys.exit(1)

    t = threading.Thread(target=serial_reader, args=(port, BAUD_RATE), daemon=True)
    t.start()

    fig, ax_map, ax_status = build_plot()

    timer = fig.canvas.new_timer(interval=REFRESH_MS)
    timer.add_callback(update_plot, fig, ax_map, ax_status)
    timer.start()

    plt.tight_layout(pad=1.5)
    plt.show()


if __name__ == "__main__":
    main()
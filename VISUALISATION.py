import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

PORT = "COM3"      # change if needed
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.1)

# ---------------- Plot setup ----------------
fig, ax = plt.subplots()
ax.set_xlim(-1, 3)
ax.set_ylim(-1, 3)
ax.set_title("Indoor Position Visualization")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")

point, = ax.plot([], [], 'bo')      # moving node
anchors_x = [0, 2, 0]
anchors_y = [0, 0, 2]
ax.plot(anchors_x, anchors_y, 'ro') # anchors

print("Listening for data...\n")

# ---------------- Update function ----------------
def update(frame):
    line = ser.readline().decode(errors="ignore").strip()

    if line.startswith("DATA"):
        try:
            _, tag, xs, ys = line.split(",")

            x = float(xs)
            y = float(ys)

            # Update plot
            point.set_data([x], [y])

            # PRINT RECEIVED DATA
            print(f"RECEIVED → Tag: {tag} | X: {x:.3f} m | Y: {y:.3f} m")

        except ValueError:
            pass

    return point,

# ---------------- Animation ----------------
ani = animation.FuncAnimation(
    fig,
    update,
    interval=200,          # 5 Hz update
    blit=True,
    cache_frame_data=False
)

plt.show()
ser.close()

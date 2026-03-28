import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

PORT="COM3"
BAUD=115200

ser=serial.Serial(PORT,BAUD,timeout=0.1)

fig,ax=plt.subplots()

ax.set_xlim(-1,5)
ax.set_ylim(-1,5)
ax.set_title("Indoor Position System")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")

anchors_x=[0,3,0]
anchors_y=[0,0,3]

ax.plot(anchors_x,anchors_y,'ro')

point,=ax.plot([],[],'bo')

status=ax.text(0.02,0.95,"Status OK",transform=ax.transAxes)

def decode_mask(mask):

    missing=[]

    if not (mask & 1):
        missing.append("A")

    if not (mask & 2):
        missing.append("B")

    if not (mask & 4):
        missing.append("C")

    if len(missing)==0:
        return "All Beacons OK"

    return "Missing: "+",".join(missing)

def update(frame):

    line=ser.readline().decode(errors="ignore").strip()

    if line.startswith("DATA"):

        parts=line.split(",")

        tag=parts[1]
        x=float(parts[2])
        y=float(parts[3])
        mask=int(parts[4])

        point.set_data([x],[y])

        status.set_text(decode_mask(mask))

        print("TAG",tag,"X",x,"Y",y,"Mask",mask)

    return point,status

ani=animation.FuncAnimation(fig,update,interval=200,blit=False)

plt.show()
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.sankey import Sankey

# Define the main process
fig, ax = plt.subplots(figsize=(12, 8))
ax.axis('off')

# Draw flowchart elements
# Define basic shapes for the flowchart
def add_rectangle(text, xy, width=3.5, height=1.5):
    return ax.add_patch(mpatches.Rectangle(xy, width, height, edgecolor="black", facecolor="lightblue", lw=1))

def add_circle(text, xy, radius=0.75):
    return ax.add_patch(mpatches.Circle(xy, radius, edgecolor="black", facecolor="lightgreen", lw=1))

def connect(p1, p2, text=""):
    ax.annotate(
        text,
        xy=p2,
        xytext=p1,
        arrowprops=dict(arrowstyle="->", lw=1.5),
        fontsize=9,
        ha="center",
    )

# Create flowchart blocks
start = (-6, 10)
setup_block = (-6, 7)
loop_decision = (-6, 4)
imu_update = (-10, 1)
pid_logic = (-6, 1)
motor_control = (-2, 1)
serial_input = (-6, -2)

# Adding nodes
add_circle("Start", start)
add_rectangle("Setup", setup_block)
add_rectangle("Loop: IMU Update?", loop_decision)
add_rectangle("IMU Update", imu_update)
add_rectangle("PID Logic", pid_logic)
add_rectangle("Motor Control", motor_control)
add_rectangle("Serial Input", serial_input)

# Connecting nodes
connect(start, setup_block, "Start setup()")
connect(setup_block, loop_decision, "Enter loop()")
connect(loop_decision, imu_update, "If IMU Update",)
connect(loop_decision, serial_input, "Else check Serial")
connect(imu_update, pid_logic, "Update Sensor Data")
connect(pid_logic, motor_control, "Compute PID Output")
connect(motor_control, loop_decision, "Go to next loop")

# Display flowchart
plt.title("Flowchart of Code Execution", fontsize=14)
plt.show()

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import argparse
import sys

# Parse arguments for COM port
parser = argparse.ArgumentParser(description='Plot IMU Data')
parser.add_argument('--port', type=str, required=True, help='COM port (e.g., COM3)')
parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
args = parser.parse_args()

try:
    # Small timeout so readline() doesn't block forever if no data
    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    print(f"Connected to {args.port} at {args.baud} baud. Waiting for data...")
except Exception as e:
    print(f"Error opening serial port: {e}")
    sys.exit(1)

# Pre-fill the data arrays with zeros so the graph draws immediately
MAX_POINTS = 500
rolls = deque([0.0]*MAX_POINTS, maxlen=MAX_POINTS)
pitches = deque([0.0]*MAX_POINTS, maxlen=MAX_POINTS)
yaws = deque([0.0]*MAX_POINTS, maxlen=MAX_POINTS)
axs = deque([0.0]*MAX_POINTS, maxlen=MAX_POINTS)
ays = deque([0.0]*MAX_POINTS, maxlen=MAX_POINTS)
azs = deque([0.0]*MAX_POINTS, maxlen=MAX_POINTS)

# Setup plotting
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

# Subplot 1: Orientation
line_roll, = ax1.plot(rolls, label='Roll (X)', color='r')
line_pitch, = ax1.plot(pitches, label='Pitch (Y)', color='g')
line_yaw, = ax1.plot(yaws, label='Yaw (Z)', color='b')
ax1.set_ylim(-180, 180)
ax1.set_xlim(0, MAX_POINTS)
ax1.set_title('IMU Orientation (Madgwick AHRS)')
ax1.set_ylabel('Degrees')
ax1.legend(loc='upper right')
ax1.grid(True)

# Subplot 2: Accelerometer
line_ax, = ax2.plot(axs, label='Accel X', color='c')
line_ay, = ax2.plot(ays, label='Accel Y', color='m')
line_az, = ax2.plot(azs, label='Accel Z', color='y')
ax2.set_ylim(-16.5, 16.5) # Based on +/- 16g range
ax2.set_xlim(0, MAX_POINTS)
ax2.set_title('Accelerometer (g)')
ax2.set_xlabel('Samples')
ax2.set_ylabel('g')
ax2.legend(loc='upper right')
ax2.grid(True)

def update(frame):
    lines_read = 0
    # Read up to 50 lines per frame to prevent the GUI from freezing
    while ser.in_waiting > 0 and lines_read < 50:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            lines_read += 1
            
            if not line:
                continue
                
            parts = line.split(',')
            if len(parts) == 6:
                r, p, y, ax_val, ay_val, az_val = map(float, parts)
                rolls.append(r)
                pitches.append(p)
                yaws.append(y)
                axs.append(ax_val)
                ays.append(ay_val)
                azs.append(az_val)
                
                # Print the latest value to the terminal
                sys.stdout.write(f"\rRoll: {r:>8.4f} | Pitch: {p:>8.4f} | Yaw: {y:>8.4f} | ax: {ax_val:>6.4f} | ay: {ay_val:>6.4f} | az: {az_val:>6.4f}")
                sys.stdout.flush()
                
        except Exception as e:
            pass

    # Update the lines with the new data
    line_roll.set_ydata(rolls)
    line_pitch.set_ydata(pitches)
    line_yaw.set_ydata(yaws)
    line_ax.set_ydata(axs)
    line_ay.set_ydata(ays)
    line_az.set_ydata(azs)
    
    return line_roll, line_pitch, line_yaw, line_ax, line_ay, line_az

# blit=True makes the animation much faster
ani = animation.FuncAnimation(fig, update, interval=20, blit=True, cache_frame_data=False)
plt.tight_layout()
plt.show()

ser.close()
print("\nDisconnected.")

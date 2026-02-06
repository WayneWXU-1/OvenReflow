import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys
import serial
from collections import deque

xsize = 100  # Number of data points visible on x-axis
SMOOTH_WINDOW = 10  # Number of samples to average for smoothing

# Configure the serial port
ser = serial.Serial(
    port='COM9',
    baudrate=57600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

# Buffer for smoothing
smooth_buffer = deque(maxlen=SMOOTH_WINDOW)

def smooth(value):
    """Apply moving average smoothing."""
    smooth_buffer.append(value)
    return sum(smooth_buffer) / len(smooth_buffer)

def data_gen():
    t = 0
    while True:
        try:
            line = ser.readline().decode('ascii').strip()
            if line:
                val = float(line)
                smoothed_val = smooth(val)
                t += 1
                yield t, val, smoothed_val
        except (ValueError, UnicodeDecodeError):
            # Skip invalid readings
            pass

def run(data):
    t, y_raw, y_smooth = data
    if t > 0:
        xdata.append(t)
        ydata_raw.append(y_raw)
        ydata_smooth.append(y_smooth)
        if t > xsize:  # Scroll to the left
            ax.set_xlim(t - xsize, t)
        line_raw.set_data(xdata, ydata_raw)
        line_smooth.set_data(xdata, ydata_smooth)
    return line_raw, line_smooth

def on_close_figure(event):
    ser.close()
    sys.exit(0)

# Setup plot
fig = plt.figure()
fig.canvas.mpl_connect('close_event', on_close_figure)
ax = fig.add_subplot(111)
line_raw, = ax.plot([], [], lw=1, color='lightcoral', alpha=0.5, label='Raw')
line_smooth, = ax.plot([], [], lw=2, color='red', label='Smoothed')
ax.set_ylim(0, 300)  # Celsius range (adjust as needed)
ax.set_xlim(0, xsize)
ax.set_xlabel('Time (samples)')
ax.set_ylabel('Temperature (°C)')
ax.set_title('Live Temperature from LM335')
ax.legend(loc='upper right')
ax.grid()
xdata, ydata_raw, ydata_smooth = [], [], []

ani = animation.FuncAnimation(fig, run, data_gen, blit=False, interval=100, repeat=False, cache_frame_data=False)
plt.show()
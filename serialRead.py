import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys
import serial
from collections import deque

import csv
import time

import smtplib
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.base import MIMEBase
from email import encoders

xsize = 100  # Number of data points visible on x-axis
SMOOTH_WINDOW = 10  # Number of samples to average for smoothing
filename = ""

# Configure the serial port
ser = serial.Serial(
    port='COM6',
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

import re

csv_data = []

# ... (rest of imports remain the same)

def data_gen():
    t = 0
    while True:
        try:
            line = ser.readline().decode('ascii').strip()
            if line:
                # Find the first sequence of digits (and optional decimal)
                match = re.search(r'(\d+(\.\d+)?)', line)
                if match:
                    val = float(match.group(1))
                    
                    # Filter crazy values (e.g., erroneous spikes > 500)
                    if 0 <= val <= 300:
                        smoothed_val = smooth(val)
                        t += 1
                        csv_data.append([t, val, smoothed_val])
                        yield t, val, smoothed_val
        except (ValueError, UnicodeDecodeError):
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
    global filename

    filename = f"oven_data_{int(time.time())}.csv"

    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Time','Temperature','Smoothed Temperature'])
        writer.writerows(csv_data)
        print(f"Data saved to {filename}")
    
    ser.close()

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

smtpObj = smtplib.SMTP('smtp.gmail.com', 587)
smtpObj.ehlo()
smtpObj.starttls()
smtpObj.login('faleksa2006@gmail.com', 'ssfa ycio kxbs zzqy')

from_address = 'faleksa2006@gmail.com'
to_address = input("Enter target email address: ")

msg = MIMEMultipart()
msg['From'] = from_address
msg['To'] = to_address
msg['Subject'] = "CSV Export - Reflow Session"
msg.attach(MIMEText("Attached is the current session data.", 'plain'))


with open(filename, "rb") as attachment:
    part = MIMEBase("application", "octet-stream")
    part.set_payload(attachment.read())
encoders.encode_base64(part)
part.add_header('Content-Disposition', f'attachment; filename= {filename}')
msg.attach(part)

smtpObj.sendmail(from_address, to_address, msg.as_string())

# Quit the SMTP session
smtpObj.quit()
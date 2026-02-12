import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys
import serial
from collections import deque
import tkinter as tk
from tkinter import simpledialog

import csv
import time

# Email imports
import smtplib
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.base import MIMEBase
from email import encoders

# --- ADDED (from File 1) ---
from flask import Flask
import threading
# ---------------------------

xsize = 100  # Number of data points visible on x-axis
SMOOTH_WINDOW = 10  # Number of samples to average for smoothing
filename = ""

latest_temp = 0.0
serial_closed = False

# Configure the serial port
# ADJUST PORT AS NEEDED
ser = serial.Serial(
    port='COM6',
    baudrate=57600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_TWO,  # maybe change to one stop bit!!
    bytesize=serial.EIGHTBITS,
    timeout=1
)

# --- ADDED (from File 1) ---
app = Flask(__name__)
ABORT_TOKEN = b'\xA5'   # single byte token

@app.route("/")
def home():
    return """  
    <!DOCTYPE html>
    <html>
    <head>
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>Oven Controller</title>
        <style>
            body {
                margin: 0;
                height: 100vh;
                display: flex;
                justify-content: center;
                align-items: center;
                background-color: #111;
                font-family: Arial, sans-serif;
                color: white;
                text-align: center;
            }
            .container { width: 100%; padding: 24px; box-sizing: border-box; }
            h1 { margin: 0 0 28px 0; font-size: 28px; font-weight: 700; }
            .status { margin: 0 0 24px 0; font-size: 16px; opacity: 0.85; }
            button {
                width: 85%;
                max-width: 420px;
                height: 130px;
                font-size: 34px;
                font-weight: 800;
                border: none;
                border-radius: 22px;
                background-color: #cc0000;
                color: white;
                box-shadow: 0 10px #7a0000;
                cursor: pointer;
                user-select: none;
                -webkit-tap-highlight-color: transparent;
            }
            button:active {
                box-shadow: 0 5px #7a0000;
                transform: translateY(5px);
            }
            button[disabled] {
                background-color: #555;
                box-shadow: none;
                transform: none;
                cursor: default;
                opacity: 0.9;
            }
        </style>
    </head>
    <body>
        <div class="container">
            <h1>Oven Controller</h1>
            <div class="status" id="status">Ready</div>
            <h2 id="tempDisplay" style="font-size:48px;margin:20px 0;">-- °C</h2>
            <button id="abortBtn" onclick="abortOven()">ABORT</button>
        </div>

        <script>
            async function abortOven() {
                const btn = document.getElementById("abortBtn");
                const status = document.getElementById("status");

                btn.disabled = true;
                btn.innerText = "SENDING...";
                status.innerText = "Sending abort request...";

                try {
                    const res = await fetch('/abort', { method: 'POST' });
                    if (!res.ok) throw new Error("Server returned " + res.status);

                    btn.innerText = "ABORT SENT";
                    status.innerText = "Abort sent to MCU";
                } catch (e) {
                    btn.disabled = false;
                    btn.innerText = "ABORT";
                    status.innerText = "Failed to send. Try again.";
                }
            }

            async function updateTemp() {
                try {
                    const res = await fetch('/data');
                    const data = await res.json();
                    const tempEl = document.getElementById("tempDisplay");
                    tempEl.innerText = data.temp.toFixed(1) + " °C";
                } catch (e) {
                    // ignore
                }
            }

            setInterval(updateTemp, 500);
        </script>
    </body>
    </html>
    """

@app.route("/data")
def data():
    global latest_temp
    return {"temp": latest_temp}

@app.route("/abort", methods=["POST"])
def abort():
    global serial_closed
    if serial_closed:
        return "SERIAL CLOSED", 409

    try:
        ser.write(ABORT_TOKEN)   # Send raw byte 0xA5
        ser.flush()
        print("ABORT token 0xA5 sent to MCU")
        return "OK"
    except Exception as e:
        print("Serial error:", e)
        return "ERROR", 500

def run_server():
    # IMPORTANT: use_reloader=False so Flask doesn't start twice
    app.run(host="0.0.0.0", port=8000, debug=False, use_reloader=False)

# Start server in background so matplotlib can run in main thread
server_thread = threading.Thread(target=run_server, daemon=True)
server_thread.start()
print("ABORT web server running on http://<YOUR_PC_IP>:8000")
# ---------------------------

# Buffer for smoothing
smooth_buffer = deque(maxlen=SMOOTH_WINDOW)

def smooth(value):
    """Apply moving average smoothing."""
    smooth_buffer.append(value)
    return sum(smooth_buffer) / len(smooth_buffer)

import re

csv_data = []

# Ask for email at start to avoid race conditions
root = tk.Tk()
root.withdraw()
recipient = simpledialog.askstring("Email", "Enter target email address:")
root.destroy()

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
                        global latest_temp
                        smoothed_val = smooth(val)
                        latest_temp = smoothed_val
                        t += 1
                        csv_data.append([t, val, smoothed_val])
                        yield t, val, smoothed_val
        except (ValueError, UnicodeDecodeError):
            pass

def run(data):
    # Unpack the yielded data
    t, y_raw, y_smooth = data
    if t > 0:
        xdata.append(t)
        # We focus on the smoothed data for the color plot
        ydata_smooth.append(y_smooth)

        # Scroll the X-axis
        if t > xsize:
            ax.set_xlim(t - xsize, t)

        points = np.column_stack((xdata, ydata_smooth))
        scat.set_offsets(points)
        scat.set_array(np.array(ydata_smooth))

    return scat,

def on_close_figure(event):
    global filename
    global serial_closed

    filename = f"oven_data_{int(time.time())}.csv"

    # Save CSV
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Time','Temperature','Smoothed Temperature'])
        writer.writerows(csv_data)
        print(f"Data saved to {filename}")

    try:
        ser.close()
    finally:
        serial_closed = True

    # Do NOT put sys.exit here, let it fall through to email code

# Setup plot
fig = plt.figure()
fig.canvas.mpl_connect('close_event', on_close_figure)
ax = fig.add_subplot(111)

scat = ax.scatter([], [], c=[], cmap='jet', vmin=20, vmax=260, s=30, label='Smoothed Temp')

# Add a Colorbar to show the scale
cbar = plt.colorbar(scat, ax=ax)
cbar.set_label('Temperature (°C)')

ax.set_ylim(0, 300)  # Celsius range
ax.set_xlim(0, xsize)
ax.set_xlabel('Time (samples)')
ax.set_ylabel('Temperature (°C)')
ax.set_title('Live Reflow Temperature')
ax.grid()

xdata, ydata_smooth = [], []

ani = animation.FuncAnimation(fig, run, data_gen, blit=False, interval=100, repeat=False, cache_frame_data=False)
plt.show()

# --- EMAIL LOGIC (Runs after window closes) ---
if filename and recipient:
    print(f"Sending email to {recipient}...")
    try:
        from_address = 'faleksa2006@gmail.com'
        to_address = recipient

        msg = MIMEMultipart()
        msg['From'] = from_address
        msg['To'] = to_address
        msg['Subject'] = "CSV Export - Reflow Session"
        msg.attach(MIMEText("Attached is the current session data.", 'plain'))

        # Attach CSV
        with open(filename, "rb") as attachment:
            part = MIMEBase("application", "octet-stream")
            part.set_payload(attachment.read())

        encoders.encode_base64(part)
        part.add_header('Content-Disposition', f'attachment; filename= {filename}')
        msg.attach(part)

        # Send
        smtpObj = smtplib.SMTP('smtp.gmail.com', 587)
        smtpObj.ehlo()
        smtpObj.starttls()
        smtpObj.login('faleksa2006@gmail.com', 'ssfa ycio kxbs zzqy')
        smtpObj.sendmail(from_address, to_address, msg.as_string())
        smtpObj.quit()
        print("Email sent successfully!")

    except Exception as e:
        print(f"Email failed: {e}")

sys.exit(0)
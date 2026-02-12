from flask import Flask
import serial

app = Flask(__name__)

# Open serial port (same settings as your working logger)
ser = serial.Serial(
    port='COM3',
    baudrate=57600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,   # 8051 Mode 1 = 1 stop bit
    bytesize=serial.EIGHTBITS,
    timeout=1
)

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
        </script>
    </body>
    </html>
    """

@app.route("/abort", methods=["POST"])
def abort():
    try:
        ser.write(ABORT_TOKEN)   # Send raw byte 0xA5
        ser.flush()
        print("ABORT token 0xA5 sent to MCU")
        return "OK"
    except Exception as e:
        print("Serial error:", e)
        return "ERROR", 500

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8000)
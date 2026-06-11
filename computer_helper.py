import serial
import time
import urllib.request
import urllib.parse

# --- YOU NEED TO CHANGE THESE TWO LINES ---
# 1. The web link from your Google Sheet (we will make this next)
WEB_APP_URL = "https://script.google.com/macros/s/AKfycbw51tywfcFrjDIKR7z4goBBYeFAuH4n6XmQB4Xb2pYmlc3Q_6i7EFS1AWDidTzG76F6nw/exec"

# 2. The name of the USB port your robot is plugged into 
# (On a Mac, it usually looks like '/dev/cu.usbmodem14101' or similar)
ROBOT_PORT = "/dev/cu.usbmodem326E355D33381" 
# ----------------------------------------

print("Starting the computer helper!")
print("Trying to connect to the robot...")

try:
    # Connect to the robot's walkie-talkie (USB port)
    robot = serial.Serial(ROBOT_PORT, 115200, timeout=1)
    print("Connected to robot! Listening for messages...")

    while True:
        # Read raw bytes from the robot
        raw_line = robot.readline()
        if not raw_line:
            time.sleep(0.1)
            continue

        # Decode normally (in case it works fine)
        line_normal = raw_line.decode('utf-8', errors='ignore').strip()

        # Decode after XORing with 3 (fixes the Mac serial bit‑flip issue)
        fixed_bytes = bytes([b ^ 3 for b in raw_line])
        line_fixed = fixed_bytes.decode('utf-8', errors='ignore').strip()

        if line_normal:
            print("Robot sent (raw):", repr(line_normal))
        if line_fixed and line_fixed != line_normal:
            print("Robot sent (fixed):", repr(line_fixed))

        # Check if the secret message is in either version
        active_line = None
        if "UPDATE_SHEET" in line_fixed:
            active_line = line_fixed
        elif "UPDATE_SHEET" in line_normal:
            active_line = line_normal

        if active_line:
            print("Heard the secret message! Updating Google Sheets...")

            # Split on the marker. The first element is usually empty or garbage.
            parts = active_line.split('UPDATE_SHEET:')
            
            import re
            
            for part in parts[1:]:  # Loop over every message in the buffer
                message_text = ''.join(ch for ch in part if ch.isprintable()).strip()
                if message_text.startswith('(!'):
                    message_text = message_text[2:].strip()

                # Extract numeric motor positions AND deltas using regex
                m = re.search(r'Left:\s*(-?\d+).*Right:\s*(-?\d+).*DLeft:\s*(-?\d+).*DRight:\s*(-?\d+)', message_text)
                if m:
                    left_val = m.group(1)
                    right_val = m.group(2)
                    dleft_val = m.group(3)
                    dright_val = m.group(4)

                    # Build a single request that sends all values.
                    payload = {
                        'left': left_val,
                        'right': right_val,
                        'dleft': dleft_val,
                        'dright': dright_val
                    }
                    url = WEB_APP_URL + "?" + urllib.parse.urlencode(payload)
                    urllib.request.urlopen(url)   # fire‑and‑forget
                    print(f"Logged row: left={left_val}, right={right_val}, dleft={dleft_val}, dright={dright_val}")
                else:
                    # Fallback – just send whatever we managed to clean
                    full_url = WEB_APP_URL + "?text=" + urllib.parse.quote(message_text)
                    try:
                        urllib.request.urlopen(full_url)
                        print(f"Success! Cell A1 now says: '{message_text}'")
                    except:
                        pass

        time.sleep(0.1)

except Exception as e:
    print("Error connecting to the robot. Make sure the port name is correct and the robot is plugged in!")
    print("Error details:", e)

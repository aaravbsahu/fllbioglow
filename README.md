# Connecting SPIKE Prime to Google Sheets

This guide explains how we set up a LEGO SPIKE Prime 3 robot to automatically update cell A1 of a Google Sheet to say **"bye"** when it finishes its program.

Since the SPIKE Hub cannot connect directly to the internet, we use a Mac computer as a "bridge" (or middleman) to listen to the robot and update the sheet.

---

## 🛠️ The Architecture

```
[ SPIKE Robot ] 
      │ (USB Serial Connection)
      ▼
[ Mac Computer (computer_helper.py) ]
      │ (Internet Request)
      ▼
[ Google Apps Script Web App ] ──▶ [ Google Sheet (Cell A1) ]
```

---

## 📋 Complete Setup Instructions

### Step 1: Set up Google Sheets and the "Secret Backdoor"
Because Google Sheets requires passwords, we created a Google Apps Script that acts as a public link to edit the sheet:

1. Open your Google Sheet.
2. In the top menu, click **Extensions** -> **Apps Script**.
3. Replace the code with this script:
   ```javascript
   function doGet(e) {
     var sheet = SpreadsheetApp.getActiveSpreadsheet().getActiveSheet();
     sheet.getRange("A1").setValue(e.parameter.text);
     return ContentService.createTextOutput("Success!");
   }
   ```
4. Click **Deploy** -> **New deployment**.
5. Choose **Web app** as the type.
6. Under **Who has access**, select **Anyone** (this is very important!).
7. Deploy it and copy the **Web app URL** provided.

---

### Step 2: Configure the Robot Code (`Project 13.llsp3`)
We modified the Python program inside the SPIKE Prime project to send a message over the USB cable after the robot says "bye":

```python
from hub import light_matrix
import runloop

async def main():
    # Write your code here
    await light_matrix.write("bye")
    print("UPDATE_SHEET")  # This sends the secret message to the computer!

runloop.run(main())
```

> [!IMPORTANT]
> **SPIKE App Auto-Save Warning:** 
> When modifying `.llsp3` files directly on disk, **always close the project in the SPIKE App first**. If you leave the project open, the SPIKE App will auto-save and overwrite your file modifications with the old code in its memory. We changed the project ID inside `manifest.json` to force the app to recognize the updated file as a new project.

---

### Step 3: Configure the Computer Helper (`computer_helper.py`)
We created a Python script on the Mac that listens to the USB port. 

1. Open `computer_helper.py` in this directory.
2. Verify the top settings are correct:
   - `WEB_APP_URL` is set to your Google Apps Script URL.
   - `ROBOT_PORT` is set to your robot's USB port (e.g., `"/dev/cu.usbmodem326E355D33381"`).

#### 🐛 The Mac USB Bit-Flipping Bug
During testing, we discovered a Mac USB-to-serial driver bug where characters coming from the robot had their last two bits flipped (XORed with `3`). 
- For example, the robot sent `UPDATE_SHEET` but the computer read it as `VSGBWF\PKFFW`.
- The helper script automatically fixes this by XORing the bytes back with `3` so it decodes clean text!

---

### Step 4: Run the System
1. Connect the robot to your Mac with the USB cable.
2. Open your Mac's **Terminal** app.
3. Make sure the serial communicator is installed:
   ```bash
   python3 -m pip install pyserial
   ```
4. **Disconnect the robot from the SPIKE App** (or close the SPIKE App completely) so it doesn't block the computer port.
5. Run the helper program:
   ```bash
   python3 Desktop/git/"fll bioglow"/computer_helper.py
   ```
6. Press the physical **Play** button on the robot. The Google Sheet will update!

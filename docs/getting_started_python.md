# Quickstart Guide: Python (MicroPython) Track

This guide will take you from a bare microcontroller to running your first motor-spinning script in under 5 minutes.

---

## 🛠️ Step 1: Physical Connections & Safety

To protect your hardware, strictly follow the safety rules:
1. **Single USB Cable Only:** Connect **one** USB-C cable to the port labeled **ST-LINK** (the programmer debugger board). Do not plug multiple USB cables into the board or chassis at the same time.
2. **Do Not Touch the Wheels:** The motors have high-ratio gearboxes. Spinning the wheels by hand will strip the gears and permanently destroy them.
3. **Turn the Battery On (If connected to chassis):** Ensure the battery is charged and the physical switch on the mouse chassis is turned **ON** so the sensors and motors are powered. *(Note: If you have removed the processor board from the chassis, it runs purely on USB power).*

---

## 💾 Step 2: Flash the MicroPython Interpreter

To run Python code, the microcontroller needs the MicroPython firmware engine. 

1. Plug the ST-LINK USB cable into your laptop.
2. A virtual flash drive (usually named **NODE_L476RG** or **DIS_L476VG**) will appear on your desktop.
3. Locate the precompiled binary in your repository:
   * **[firmware/binaries/micropython.bin](../firmware/binaries/micropython.bin)**
4. **Drag and drop** `micropython.bin` directly onto that virtual flash drive.
5. The LED on the programmer will blink rapidly for a few seconds and then turn solid. The MicroPython engine is now flashed!

### 🔌 Alternative: Flash over USB OTG using DFU (No ST-Link)

If you are programmed to run without an ST-Link programmer, you can flash the firmware binary directly over the USB OTG port using the built-in system bootloader:

1. **Install `dfu-util`:**
   * **macOS (Homebrew):** Run `brew install dfu-util` in your terminal.
   * **Windows:** Download the binary from the official website and add it to your PATH.
2. **Enter DFU Bootloader Mode:**
   * If MicroPython is running, open the REPL and execute:
     ```python
     import machine
     machine.bootloader()
     ```
   * If the board is unprogrammed or locked, bridge the `BOOT0` header pin to `3V3` (using a jumper or screwdriver tip) while pressing and releasing the **RESET** button.
3. **Flash the Firmware:**
   * Connect your laptop directly to the **USB OTG** port on the board.
   * Run the central deployment tool:
     ```bash
     python3 tools/deploy.py --engine micropython --flash
     ```
     *(The tool will auto-detect the DFU state and flash the binary via `dfu-util` over your OTG cable).*

---

## 📂 Step 3: Mount the USB Drive (`UCT_MMOUSE`)

Once MicroPython is flashed, the board acts as a USB storage drive named **`UCT_MMOUSE`**.

1. Press the black **RESET** button on the processor board.
2. Within 2–3 seconds, a new USB flash drive named **`UCT_MMOUSE`** will mount automatically on your desktop.
3. Open the drive. You will see three default files:
   * `boot.py`: Configures the USB connection.
   * `main.py`: The placeholder script that executes when the board boots.
   * `README.txt`: General details about the partition.

---

## 💻 Step 4: Interact with the Python REPL (Interactive Console)

You can run Python code live on the board using the interactive REPL.

1. Open your terminal application.
2. Connect to the board's serial interface using `mpremote` (recommended):
   ```bash
   pip install mpremote
   mpremote repl
   ```
   *(Alternative: use screen `screen /dev/cu.usbmodem* 115200`)*
3. Press **`Enter`** or **`Ctrl+C`**. You will see the MicroPython prompt:
   ```python
   >>> 
   ```
4. Try typing a command:
   ```python
   >>> import machine
   >>> led = machine.Pin('PC13', machine.Pin.OUT)
   >>> led.value(1)  # Onboard LED turns ON
   >>> led.value(0)  # Onboard LED turns OFF
   ```

---

## 🚀 Step 5: Write and Deploy Your First Script

Let's make the motors spin!

1. Open the **`UCT_MMOUSE`** drive on your computer.
2. Open **`main.py`** in a text editor (e.g. VS Code, TextEdit).
3. Replace the placeholder content with the following test script:

```python
import uct_mouse
import time

def main():
    # Initialize connection
    uct_mouse.init()

    # Set motor polarity (Default: 1, 1)
    uct_mouse.set_polarity(1, 1)

    print("--- Running Motor Test Script ---")

    # Spin motors forward at 40% PWM for 1.5 seconds
    uct_mouse.set_motors(40, 40)
    uct_mouse.delay_ms(1500)

    # Stop the motors
    uct_mouse.set_motors(0, 0)

if __name__ == "__main__":
    main()
```

5. Save the file.
6. Press the black **RESET** button on the processor board. The motors will spin forward for 1.5 seconds and stop!

> [!IMPORTANT]
> **Do NOT copy the file `uct_mouse.py` onto the `UCT_MMOUSE` USB drive.**
> 
> * The `uct_mouse` module is **compiled directly into the MicroPython firmware** as a native, built-in C module.
> * The file `uct_mouse.py` in the repository is a **PC-side mock wrapper** designed solely to run scripts on your computer (directing them to the Simulink simulator).
> * If you copy `uct_mouse.py` to the USB drive, it will override the native C library and cause your scripts on the board to crash immediately because the microcontroller cannot run PC networking libraries (`socket`, `subprocess`).

---

## 🔍 Troubleshooting

* **`UCT_MMOUSE` volume not mounting:**
  If the drive does not mount, check if the red power LED on the board is on. Try unplugging the USB cable, pressing the RESET button while plugging it back in, or using a different USB port.
* **Traceback / Error Screen:**
  If your script crashes, a stack trace is written to `error_log.txt` on the USB drive. You can open that file on your computer to see why your code failed!

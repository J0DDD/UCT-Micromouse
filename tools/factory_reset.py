#!/usr/bin/env python3
import sys
import time
import argparse
import subprocess
import shutil
import os
import serial
import serial.tools.list_ports

def detect_port():
    mpy_port = None
    stlink_port = None
    
    for p in serial.tools.list_ports.comports():
        if "ST-Link" in p.description or "STLink" in p.description or (p.vid == 0x0483 and p.pid in (0x374b, 0x3752)) or "usbmodem" in p.device:
            stlink_port = p.device
        elif p.vid == 0xf055 and p.pid == 0x9800:
            mpy_port = p.device

    if stlink_port:
        return stlink_port
    elif mpy_port:
        return mpy_port
    return None

def find_st_flash_cmd():
    for cmd in ["st-flash", "/opt/homebrew/bin/st-flash", "/usr/local/bin/st-flash", "/opt/local/bin/st-flash"]:
        if shutil.which(cmd) or os.path.exists(cmd):
            return cmd
    return None

def main():
    parser = argparse.ArgumentParser(description="UCT Micromouse Hardware Factory Reset Tool")
    parser.add_argument(
        "--engine", "-e",
        choices=["micropython", "pikascript", "simulink"],
        default="micropython",
        help="Select which firmware engine to flash after erasing (default: micropython)"
    )
    parser.add_argument(
        "--port", "-p",
        help="Serial port of the mouse (auto-detected if omitted)"
    )
    args = parser.parse_args()

    print("=== UCT Micromouse Factory Reset ===")
    print("This tool will completely erase the external SPI flash (clearing all telemetry logs")
    print("and internal FAT filesystem/scripts) and wipe/reflash the STM32 internal flash.")
    print("--------------------------------------------------------------------------------")

    # Step 1: Attempt to erase external SPI flash via MicroPython REPL or C-Kernel command
    ports_to_try = [args.port] if args.port else []
    if not ports_to_try:
        for p in serial.tools.list_ports.comports():
            if "ST-Link" in p.description or "STLink" in p.description or (p.vid == 0x0483 and p.pid in (0x374b, 0x3752)) or "usbmodem" in p.device:
                ports_to_try.append(p.device)
            elif p.vid == 0xf055 and p.pid == 0x9800:
                ports_to_try.append(p.device)
    
    ports_to_try = list(dict.fromkeys(ports_to_try))
    if ports_to_try:
        for port in ports_to_try:
            print(f"[1/4] Connecting to {port} to request SPI Flash Chip Erase...")
            try:
                ser = serial.Serial(port, 115200, timeout=1.0)
                ser.reset_input_buffer()
                # 1. Interrupt any running script and exit raw REPL
                ser.write(b'\x03\x03\x02')
                time.sleep(0.1)
                
                # 2. Try MicroPython REPL erase command
                ser.write(b'\r\nimport uct_mouse; uct_mouse.erase_flash()\r\n')
                time.sleep(0.1)
                
                # 3. Try C-Kernel JSON erase command
                ser.write(b'\r\n{"c":{"erase":1}}\r\n')
                time.sleep(0.2)
                ser.close()
                print("      SPI flash erase requested. Waiting 3s for completion...")
                time.sleep(3.0)
                print("      SPI flash erase command sent successfully.")
            except Exception as e:
                print(f"      Note: Could not send erase on {port} ({e}).")
    else:
        print("[1/4] Serial port not detected. Skipping serial SPI flash command.")
        print("      (External SPI flash will be formatted by MicroPython on boot).")

    # Step 2: Locate st-flash utility
    st_flash_cmd = find_st_flash_cmd()
    if not st_flash_cmd:
        print("\nError: 'st-flash' utility not found. Please install the stlink utilities.")
        print("On macOS: 'brew install stlink'")
        print("On Ubuntu/Debian: 'sudo apt install stlink-tools'")
        sys.exit(1)

    # Step 3: Erase internal STM32 Flash
    print("\n[2/4] Erasing internal STM32 microcontroller flash...")
    try:
        subprocess.run([st_flash_cmd, "erase"], check=True)
        print("      Success: Internal flash completely wiped.")
    except subprocess.CalledProcessError as e:
        print(f"      Error: Failed to erase internal flash ({e}).")
        print("      Please check your USB cables, power switch, and ensure ST-Link is connected.")
        sys.exit(1)

    # Step 4: Reflash target firmware binary
    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.abspath(os.path.join(script_dir, ".."))
    
    bin_name = f"{args.engine}.bin"
    bin_path = os.path.join(repo_root, "firmware", "binaries", bin_name)
    
    if not os.path.exists(bin_path):
        print(f"\nError: Target firmware binary not found at: {bin_path}")
        print("Please build the firmware first or check the repository path.")
        sys.exit(1)

    print(f"\n[3/4] Reflashing fresh '{args.engine}' firmware binary...")
    try:
        subprocess.run([st_flash_cmd, "--reset", "write", bin_path, "0x08000000"], check=True)
        print(f"      Success: Firmware '{args.engine}' written to 0x08000000.")
    except subprocess.CalledProcessError as e:
        print(f"      Error: Failed to write binary to flash ({e}).")
        sys.exit(1)

    # Step 5: Formatting and setup information
    print("\n[4/4] Finalizing factory reset...")
    if args.engine == "micropython":
        print("\n*** INFO: MicroPython interpreter is now flashed. ***")
        print("On first boot, MicroPython will automatically format the external SPI flash")
        print("with a clean FAT partition structure, and populate the default 'boot.py'")
        print("and 'main.py' files. This takes about 2-3 seconds after rebooting.")
    elif args.engine == "pikascript":
        print("\n*** INFO: PikaScript interpreter is now flashed. ***")
        print("Deploy your user main.py file using the standard deploy command:")
        print("   python tools/deploy.py --engine pikascript --script workspace/main.py")
    else:
        print("\n*** INFO: Simulink firmware is now flashed. ***")

    print("\nFactory Reset Complete! The mouse has been returned to a clean, uniform slate.")

if __name__ == "__main__":
    main()

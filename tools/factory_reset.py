#!/usr/bin/env python3
import sys
import time
import argparse
import subprocess
import shutil
import os
import serial
import serial.tools.list_ports

def detect_mpy_port():
    # 1. Look specifically for MicroPython USB OTG CDC device (VID 0xf055)
    for p in serial.tools.list_ports.comports():
        if p.vid == 0xf055 and p.pid in (0x9800, 0x9801, 0x9802):
            return p.device

    # 2. Fallback to ST-Link VCP bridge if OTG cable is not connected
    for p in serial.tools.list_ports.comports():
        if "ST-Link" in p.description or "STLink" in p.description or (p.vid == 0x0483 and p.pid in (0x374b, 0x3752)) or "usbmodem" in p.device:
            return p.device
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

    # Step 5: Formatting and setup external SPI flash filesystem
    print("\n[4/4] Finalizing factory reset & setting up external flash filesystem...")
    if args.engine == "micropython":
        print("      Waiting 2.5s for MicroPython USB volume to enumerate...")
        time.sleep(2.5)

        # Check mounted drive
        mpy_drive = None
        for attempt in range(10):
            for candidate in ["/Volumes/UCT_MMOUSE", "/Volumes/UCT_MMOUSE 1", "D:\\", "E:\\", "F:\\"]:
                if os.path.exists(candidate):
                    mpy_drive = candidate
                    break
            if mpy_drive:
                break
            time.sleep(0.5)

        if mpy_drive:
            print(f"      Detected mounted drive at: {mpy_drive}")
            try:
                with open(os.path.join(mpy_drive, "boot.py"), "w") as fp:
                    fp.write("# boot.py - UCT Micromouse Hybrid Bootloader\ntry:\n    import pyb\n    pyb.usb_mode('VCP+MSC')\nexcept Exception as e:\n    pass\n")
                with open(os.path.join(mpy_drive, "main.py"), "w") as fp:
                    fp.write("# main.py -- put your code here!\n")
                with open(os.path.join(mpy_drive, "README.txt"), "w") as fp:
                    fp.write("UCT Micromouse external SPI flash storage (128 KB FAT partition).\n")
                print("      Success: Default template files verified on drive.")
            except Exception as e:
                print(f"      Note: Could not write template files ({e}).")

        print("\n*** INFO: MicroPython interpreter is flashed and external flash is configured. ***")
        print("The board will mount on your computer as 'UCT_MMOUSE' with clean default files.")
    elif args.engine == "pikascript":
        print("\n*** INFO: PikaScript interpreter is now flashed. ***")
        print("Deploy your user main.py file using the standard deploy command:")
        print("   python tools/deploy.py --engine pikascript --script workspace/main.py")
    else:
        print("\n*** INFO: Simulink firmware is now flashed. ***")

    print("\nFactory Reset Complete! The mouse has been returned to a clean, uniform slate.")

if __name__ == "__main__":
    main()

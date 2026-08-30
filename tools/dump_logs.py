#!/usr/bin/env python3
import sys
import time
import argparse
import serial
import serial.tools.list_ports

def detect_ports():
    mpy_port = None
    stlink_port = None
    
    for p in serial.tools.list_ports.comports():
        # Check for ST-Link virtual COM port
        if "ST-Link" in p.description or "STLink" in p.description or (p.vid == 0x0483 and p.pid in (0x374b, 0x3752)):
            stlink_port = p.device
        # Check for MicroPython VCP OTG port
        elif p.vid == 0xf055 and p.pid in (0x9800, 0x9801, 0x9802):
            mpy_port = p.device
        # Fallback parsing for generic usbmodem devices
        elif "usbmodem" in p.device:
            if "Pyboard" in p.description or "Virtual Comm Port" in p.description:
                mpy_port = p.device
            else:
                stlink_port = p.device
                
    return stlink_port, mpy_port

def attempt_dump(port, is_mpy=False):
    """Triggers and reads log dump from the specified port."""
    print(f"Connecting to port ({port}) at 115200 baud...")
    try:
        ser = serial.Serial(port, 115200, timeout=1.0)
    except Exception as e:
        print(f"Error opening serial port: {e}")
        return None

    # Wait for any active telemetry streams to stabilize, then send Ctrl+C to get a clean prompt
    ser.write(b'\x03')
    time.sleep(0.1)
    ser.write(b'\x03')
    time.sleep(0.1)
    ser.reset_input_buffer()

    print("Requesting log dump from C-Kernel...")
    if is_mpy:
        # MicroPython prints directly over the console stdout (OTG port)
        ser.write(b'\r\nimport uct_mouse; uct_mouse.dump_logs()\r\n')
    else:
        # PikaScript/C-Kernel prints over USART1 (ST-Link VCP)
        ser.write(b'\r\nimport uct_mouse; uct_mouse.dump_logs()\r\n')
        time.sleep(0.1)
        ser.write(b'\r\n{"c":{"dump":1}}\r\n')
    
    lines = []
    started = False
    finished = False
    start_time = time.time()
    timeout = 10.0 # 10s max read timeout

    while time.time() - start_time < timeout:
        try:
            line_bytes = ser.readline()
            if not line_bytes:
                continue
            start_time = time.time() # Reset inactivity timeout on receiving data
            line = line_bytes.decode('utf-8', errors='ignore').strip()
            
            if "--- START LOG DUMP ---" in line or line.startswith("{"):
                if not started:
                    print("Dump started...")
                    started = True
                if "--- START LOG DUMP ---" in line:
                    continue
            elif "--- END LOG DUMP ---" in line:
                print("Dump finished successfully.")
                finished = True
                break
            
            if started:
                # Filter out raw trailing padding spaces or empty lines
                line_clean = line.strip()
                if line_clean:
                    lines.append(line_clean)
        except Exception as e:
            print(f"Error reading stream: {e}")
            break

    ser.close()
    
    if started and finished:
        return lines
    return None

def main():
    parser = argparse.ArgumentParser(description="UCT Micromouse Serial Log Extractor")
    parser.add_argument("-p", "--port", help="Serial port of the mouse (auto-detected if omitted)")
    parser.add_argument("-o", "--output", default="run_log.jsonl", help="Output file path (default: run_log.jsonl)")
    args = parser.parse_args()

    stlink_port, mpy_port = detect_ports()

    # Determine active port based on connected cables and preferences
    port = args.port
    is_mpy = False
    
    if not port:
        if mpy_port:
            print("[MicroPython OTG Mode] Detected MicroPython OTG connection.")
            port = mpy_port
            is_mpy = True
        elif stlink_port:
            print("[ST-Link Mode] Detected ST-Link debug connection.")
            port = stlink_port
            is_mpy = False
            
    if not port:
        print("Error: No compatible serial device detected!")
        print("Please ensure your mouse is connected via USB (OTG or ST-Link).")
        sys.exit(1)

    lines = attempt_dump(port, is_mpy)

    if lines is None:
        print("\nError: Log dump failed.")
        print("Make sure:")
        print("  1. The mouse is powered on.")
        print("  2. You have run a demo that generated log data.")
        sys.exit(1)

    # Write captured telemetry lines to output file
    with open(args.output, "w") as f:
        for l in lines:
            f.write(l + "\n")
    print(f"Saved {len(lines)} log records to: {args.output}")

if __name__ == "__main__":
    main()

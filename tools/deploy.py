import os
import sys
import shutil
import subprocess
import glob
import argparse

def find_stlink_drive():
    """Finds the ST-Link mass storage drive on Mac/Windows/Linux."""
    if sys.platform == 'darwin':
        drives = glob.glob('/Volumes/NOD*') + glob.glob('/Volumes/*STLINK*')
    elif sys.platform == 'win32':
        import string
        from ctypes import windll
        drives = []
        bitmask = windll.kernel32.GetLogicalDrives()
        for letter in string.ascii_uppercase:
            if bitmask & 1:
                # Rough heuristic for Windows; ideally check volume label
                if os.path.exists(f"{letter}:\\DETAILS.TXT"):
                    drives.append(f"{letter}:\\")
            bitmask >>= 1
    else:
        drives = glob.glob('/media/*/NOD*') + glob.glob('/run/media/*/NOD*')
        
    return drives[0] if drives else None

def find_micropython_drive():
    """Finds the MicroPython virtual USB drive on Mac/Windows/Linux."""
    if sys.platform == 'darwin':
        drives = (
            glob.glob('/Volumes/UCT-MICROMO*') +
            glob.glob('/Volumes/UCT_MICROMO*') +
            glob.glob('/Volumes/UCT-MICROMOUSE*') +
            glob.glob('/Volumes/PYB*')
        )
    elif sys.platform == 'win32':
        import string
        import ctypes
        drives = []
        kernel32 = ctypes.windll.kernel32
        volumeNameBuffer = ctypes.create_unicode_buffer(1024)
        bitmask = kernel32.GetLogicalDrives()
        for letter in string.ascii_uppercase:
            if bitmask & 1:
                drive_path = f"{letter}:\\"
                res = kernel32.GetVolumeInformationW(
                    drive_path, volumeNameBuffer, ctypes.sizeof(volumeNameBuffer),
                    None, None, None, None, 0
                )
                if res:
                    label = volumeNameBuffer.value.upper()
                    if "UCT-MICRO" in label or "UCT_MICRO" in label or "PYB" in label:
                        drives.append(drive_path)
            bitmask >>= 1
    else:
        drives = (
            glob.glob('/media/*/*UCT-MICROMO*') +
            glob.glob('/run/media/*/*UCT-MICROMO*') +
            glob.glob('/media/*/*UCT_MICROMO*') +
            glob.glob('/run/media/*/*UCT_MICROMO*') +
            glob.glob('/media/*/*PYB*') +
            glob.glob('/run/media/*/*PYB*')
        )
    return drives[0] if drives else None

def find_dfu_util_cmd():
    """Finds the dfu-util command path."""
    for cmd in ["dfu-util", "/opt/homebrew/bin/dfu-util", "/usr/local/bin/dfu-util", "/opt/local/bin/dfu-util"]:
        if shutil.which(cmd) or os.path.exists(cmd):
            return cmd
    return None

def find_st_flash_cmd():
    """Finds the st-flash command path."""
    for cmd in ["st-flash", "/opt/homebrew/bin/st-flash", "/usr/local/bin/st-flash", "/opt/local/bin/st-flash"]:
        if shutil.which(cmd) or os.path.exists(cmd):
            return cmd
    return None

def is_dfu_device_connected(dfu_util_cmd):
    """Checks if an STM32 DFU device is currently connected."""
    try:
        res = subprocess.run([dfu_util_cmd, "-l"], capture_output=True, text=True)
        return "0483:df11" in res.stdout
    except Exception:
        return False

def flash_firmware(central_bin_path):
    """Flashes the firmware binary onto the board.
    Tries USB DFU via dfu-util first, then st-flash, and falls back to ST-Link USB mass storage copy.
    """
    dfu_util_cmd = find_dfu_util_cmd()
    if dfu_util_cmd and is_dfu_device_connected(dfu_util_cmd):
        print(f"Using direct USB DFU flash via '{dfu_util_cmd}' (over USB OTG)...")
        try:
            # 0x08000000 is the flash start address. :leave tells dfu-util to jump to application after programming.
            subprocess.run([
                dfu_util_cmd, 
                "-a", "0", 
                "-d", "0483:df11", 
                "--dfuse-address", "0x08000000:leave", 
                "-D", central_bin_path
            ], check=True)
            print("Success! Firmware flashed via USB DFU. Board reset triggered.")
            return True
        except (subprocess.CalledProcessError, FileNotFoundError) as e:
            print(f"Warning: USB DFU flashing failed: {e}")
            print("Falling back to other flashing methods...")

    st_flash_cmd = find_st_flash_cmd()
    if st_flash_cmd:
        print(f"Using direct SWD flash via '{st_flash_cmd}' (fast & reliable)...")
        try:
            subprocess.run([st_flash_cmd, "--reset", "write", central_bin_path, "0x08000000"], check=True)
            print("Success! Firmware flashed via SWD. Board reset triggered.")
            return True
        except (subprocess.CalledProcessError, FileNotFoundError) as e:
            print(f"Warning: Direct flashing via st-flash failed: {e}")
            print("Falling back to USB mass storage copy method...")
            
    # Fallback: Find the ST-Link Mass Storage Drive
    drive = find_stlink_drive()
    if not drive:
        print("Error: Could not find ST-Link USB drive or st-flash. Is the board plugged in?")
        sys.exit(1)
        
    print(f"ST-Link found at {drive}. Flashing via USB mass storage copy (slower, may hang)...")
    try:
        if sys.platform == 'darwin':
            dest_path = os.path.join(drive, "firmware.bin")
            with open(central_bin_path, "rb") as f_src, open(dest_path, "wb") as f_dst:
                f_dst.write(f_src.read())
        else:
            shutil.copy(central_bin_path, drive)
        print("Success! Firmware copied to drive. Mouse will automatically reboot.")
        return True
    except Exception as e:
        print(f"Error copying to mass storage drive: {e}")
        sys.exit(1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="UCT Micromouse Firmware and Script Deployer")
    parser.add_argument(
        "--engine", "-e",
        choices=["pikascript", "micropython", "simulink"],
        default="pikascript",
        help="Select the firmware engine to deploy (default: pikascript)"
    )
    parser.add_argument(
        "--flash", "-f",
        action="store_true",
        help="Flash the engine's compiled C firmware binary onto the board using the ST-Link drive (always happens for pikascript and simulink)."
    )
    parser.add_argument(
        "--script-only", "-o",
        action="store_true",
        help="Directly write the Python script to the STM32 flash page at 0x08078000 using st-flash. Bypasses firmware compilation and runs in <100ms. (Only for PikaScript)."
    )
    parser.add_argument(
        "--script", "-s",
        default=None,
        help="Path to a specific python script to deploy as main.py. If omitted, mirrors the entire --src-dir."
    )
    parser.add_argument(
        "--src-dir", "-d",
        default="workspace",
        help="Path to the dedicated python development folder to mirror to the mouse (default: workspace)"
    )
    args = parser.parse_args()

    print("=== UCT Micromouse Firmware Deployer ===")
    print(f"Selected engine: {args.engine.upper()}")
    
    # Dynamically resolve paths so the script can be run from anywhere
    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.abspath(os.path.join(script_dir, ".."))

    # Define files/folders to ignore during deployment
    ignore_names = {"deploy", "ekf_research", "attic", "build", "matlab", "external", "tools", ".git", "__pycache__"}
    ignore_exts = {".pdf", ".zip", ".docx", ".md", ".slx", ".slxc", ".bin", ".elf", ".map", ".mp4"}

    # Resolve target paths
    target_script = None
    target_dir = None
    
    if args.script:
        target_script = os.path.abspath(args.script)
    else:
        target_dir = os.path.abspath(args.src_dir)
        
    if args.engine in ["pikascript", "micropython"]:
        if target_script and not os.path.exists(target_script):
            print(f"Error: Target Python script not found at {target_script}")
            sys.exit(1)
        elif not target_script and not os.path.exists(target_dir):
            print(f"Error: Target Python directory not found at {target_dir}")
            sys.exit(1)

    if args.engine == "pikascript":
        # PikaScript requires a single main.py entry point
        pika_target = target_script if target_script else os.path.join(target_dir, "main.py")
        if not os.path.exists(pika_target):
            print(f"Error: PikaScript requires a single entry point script. Could not find {pika_target}")
            sys.exit(1)
            
        # Run static compatibility check (PikaLint) before deploying
        try:
            sys.path.append(os.path.join(repo_root, "tools"))
            from pikalint import lint_file
            success, error_count = lint_file(pika_target, script_only=args.script_only)
            if not success:
                print("[Deploy Aborted] Please fix the compatibility errors above before flashing the board.")
                sys.exit(1)
        except Exception as e:
            print(f"Warning: Could not run PikaLint check: {e}")
            
        if args.script_only:
            # === SCRIPT ONLY MODE ===
            print(f"=== Script-Only Flash Mode ===")
            print(f"Target script: {os.path.basename(pika_target)}")
            
            with open(pika_target, "rb") as f_in:
                py_content = f_in.read() + b"\x00"
                
            temp_bin = os.path.join(repo_root, "build", "script_only.bin")
            os.makedirs(os.path.dirname(temp_bin), exist_ok=True)
            with open(temp_bin, "wb") as f_out:
                f_out.write(py_content)
                
            print("Flashing Python script directly to 0x08078000 (Page 240)...")
            st_flash_cmd = find_st_flash_cmd()
            if not st_flash_cmd:
                print("Error: 'st-flash' utility not found. Please install stlink (e.g. 'brew install stlink' on Mac).")
                sys.exit(1)
            try:
                subprocess.run([st_flash_cmd, "--reset", "write", temp_bin, "0x08078000"], check=True)
                print("Success! Script flashed in <100ms. Board reset triggered.")
            except subprocess.CalledProcessError as e:
                print(f"Error: Direct flashing failed! Details: {e}")
                sys.exit(1)
            sys.exit(0)
            
        # === PIKASCRIPT ENGINE FLOW ===
        print(f"[1/3] Bundling {os.path.basename(pika_target)} and compiling firmware...")
        
        # Copy user script to PikaScript directory and precompile
        shutil.copy(pika_target, os.path.join(repo_root, "firmware", "src", "pikascript", "main.py"))
        
        print("    -> Running PikaScript Pre-compiler...")
        pika_dir = os.path.join(repo_root, "firmware", "src", "pikascript")
        tools_dir = os.path.join(repo_root, "tools")
        if sys.platform == 'darwin':
            precompiler = os.path.join(tools_dir, "rust-msc-mac")
        elif sys.platform == 'win32':
            precompiler = os.path.join(tools_dir, "rust-msc-win10.exe")
        else:
            precompiler = os.path.join(tools_dir, "rust-msc-linux")
            
        try:
            subprocess.run([precompiler], cwd=pika_dir, check=True)
        except FileNotFoundError:
            print(f"Error: Could not find PikaScript pre-compiler '{precompiler}' in {pika_dir}")
            sys.exit(1)
            
        # Convert the target script into a C-string header to guarantee it gets compiled into the binary
        print(f"    -> Embedding {os.path.basename(pika_target)} into C-Kernel...")
        header_path = os.path.join(repo_root, "firmware", "src", "kernel", "inc", "student_code.h")
        with open(pika_target, "r") as f_py, open(header_path, "w") as f_h:
            f_h.write("#ifndef STUDENT_CODE_H\n#define STUDENT_CODE_H\n")
            f_h.write('const char* student_python_code = \n')
            for line in f_py:
                escaped = line.replace('\\', '\\\\').replace('"', '\\"').replace('\n', '\\n')
                f_h.write(f'"{escaped}"\n')
            f_h.write(";\n#endif\n")
            
        try:
            print("    -> Configuring CMake...")
            subprocess.run(["cmake", "-S", "firmware", "-B", "firmware/build"], cwd=repo_root, check=True)
            print("    -> Building PikaScript firmware target...")
            subprocess.run(["cmake", "--build", "firmware/build", "--target", "pikascript_firmware"], cwd=repo_root, check=True)
        except subprocess.CalledProcessError:
            print("Build failed! Check your C-Kernel and PikaScript bindings.")
            sys.exit(1)
        
        bin_path = os.path.join(repo_root, "firmware", "build", "pikascript_firmware.bin")
        untracked_bin_path = os.path.join(repo_root, "build", "bin", "pikascript.bin")
        tracked_bin_path = os.path.join(repo_root, "firmware", "binaries", "pikascript.bin")
        
        if not os.path.exists(bin_path):
            print(f"Error: Compiled firmware not found at {bin_path}")
            sys.exit(1)
        os.makedirs(os.path.dirname(untracked_bin_path), exist_ok=True)
        shutil.copy(bin_path, untracked_bin_path)
        print(f"    -> Copied compiled firmware to {untracked_bin_path}")
            
        # Flash the compiled firmware onto the board
        print(f"[2/3] Flashing PikaScript firmware...")
        flash_firmware(untracked_bin_path)
        print("[3/3] Success! Mouse will automatically reboot and run main.py.")

    elif args.engine == "simulink":
        # === SIMULINK ENGINE FLOW ===
        # Re-route model build outputs to central build/ directory to keep repo clean
        simulink_build_dir = os.path.join(repo_root, "build", "UCT_KDeploy_ert_rtw")
        untracked_bin_path = os.path.join(repo_root, "build", "bin", "simulink.bin")
        tracked_bin_path = os.path.join(repo_root, "firmware", "binaries", "simulink.bin")
        
        # Compile if not present or requested
        print("[1/2] Preparing Simulink firmware binary...")
        if not os.path.exists(simulink_build_dir) or args.flash:
            # We compile by invoking the Simulink build helper script
            print("    -> Compiling Simulink model code...")
            compile_script = os.path.join(repo_root, "tools", "compile_simulink_pc.py")
            if os.path.exists(compile_script):
                subprocess.run([sys.executable, compile_script], check=True)
            else:
                print("Error: Simulink compiler script not found!")
                sys.exit(1)

        sim_bin = os.path.join(simulink_build_dir, "UCT_KDeploy.bin")
        if not os.path.exists(sim_bin):
            # Fall back to checking standard directories
            sim_bin = os.path.join(repo_root, "UCT_KDeploy_ert_rtw", "UCT_KDeploy.bin")
            
        if not os.path.exists(sim_bin):
            if os.path.exists(tracked_bin_path):
                print(f"Using precompiled release binary: {tracked_bin_path}")
                active_bin_path = tracked_bin_path
            else:
                print(f"Error: Compiled Simulink binary not found. Please build model first.")
                sys.exit(1)
        else:
            os.makedirs(os.path.dirname(untracked_bin_path), exist_ok=True)
            shutil.copy(sim_bin, untracked_bin_path)
            print(f"    -> Copied compiled binary to {untracked_bin_path}")
            active_bin_path = untracked_bin_path

        # Flash the compiled firmware onto the board
        print(f"[2/2] Flashing Simulink firmware...")
        flash_firmware(active_bin_path)
        print("Success! Simulink firmware is flashed. The board will automatically reboot and execute.")

    else:
        # === MICROPYTHON ENGINE FLOW ===
        if args.flash:
            # --- Flashing the MicroPython C-Firmware ---
            print("[1/2] Preparing MicroPython firmware binary...")
            mpy_bin_path = os.path.join(
                repo_root, "external", "micropython", "ports", "stm32", 
                "build-UCT_MICROMOUSE", "firmware.bin"
            )
            untracked_bin_path = os.path.join(repo_root, "build", "bin", "micropython.bin")
            tracked_bin_path = os.path.join(repo_root, "firmware", "binaries", "micropython.bin")
            
            # Try compiling the firmware
            print("    -> Compiling MicroPython firmware...")
            mpy_ports_dir = os.path.join(repo_root, "external", "micropython", "ports", "stm32")
            symlink_path = os.path.join(mpy_ports_dir, "boards", "UCT_MICROMOUSE")
            
            created_symlink = False
            build_success = False
            try:
                if not os.path.exists(symlink_path):
                    # Create symlink: boards/UCT_MICROMOUSE -> ../../../../../firmware/src/micropython/boards/UCT_MICROMOUSE
                    target_rel_path = os.path.join("..", "..", "..", "..", "..", "firmware", "src", "micropython", "boards", "UCT_MICROMOUSE")
                    os.symlink(target_rel_path, symlink_path)
                    created_symlink = True
                
                subprocess.run(["make", "BOARD=UCT_MICROMOUSE"], cwd=mpy_ports_dir, check=True)
                build_success = True
            except Exception as e:
                print(f"Compilation failed: {e}")
                print("Checking for existing precompiled binaries...")
            finally:
                if created_symlink and os.path.exists(symlink_path):
                    try:
                        os.remove(symlink_path)
                    except Exception:
                        pass
                        
            if build_success and os.path.exists(mpy_bin_path):
                os.makedirs(os.path.dirname(untracked_bin_path), exist_ok=True)
                shutil.copy(mpy_bin_path, untracked_bin_path)
                print(f"    -> Copied compiled firmware to {untracked_bin_path}")
                active_bin_path = untracked_bin_path
            else:
                # Fallback checks
                if os.path.exists(untracked_bin_path):
                    print(f"Using locally cached build: {untracked_bin_path}")
                    active_bin_path = untracked_bin_path
                elif os.path.exists(tracked_bin_path):
                    print(f"Using precompiled release binary: {tracked_bin_path}")
                    active_bin_path = tracked_bin_path
                else:
                    print("Error: No MicroPython binary found to flash!")
                    sys.exit(1)
            
            # Flash the compiled firmware onto the board
            print(f"[2/2] Flashing MicroPython firmware...")
            flash_firmware(active_bin_path)
            print("Success! MicroPython interpreter is flashed. The board will reboot and mount as a USB drive shortly.")
            
        else:
            # --- Deploying Python Scripts via VCP (mpremote) ---
            print("[1/2] Connecting to MicroPython via Serial (mpremote)...")
            
            # Dynamically detect MicroPython port to avoid ST-Link VCP conflicts
            mpy_port = None
            try:
                import serial.tools.list_ports
                for p in serial.tools.list_ports.comports():
                    if p.vid == 0xf055 and p.pid in (0x9800, 0x9801, 0x9802):
                        mpy_port = p.device
                        break
                if not mpy_port:
                    for p in serial.tools.list_ports.comports():
                        # Exclude 0xf055 from fallback to ensure we only pick the real ST-Link VCP
                        if p.vid != 0xf055 and ("ST-Link" in p.description or "STLink" in p.description or (p.vid == 0x0483 and p.pid == 0x374b) or "usbmodem" in p.device):
                            mpy_port = p.device
                            break
            except Exception:
                pass
                
            mpremote_cmd = [sys.executable, "-m", "mpremote"]
            if mpy_port:
                print(f"    -> Detected MicroPython on {mpy_port}")
                mpremote_cmd += ["connect", mpy_port]
            try:
                subprocess.run(mpremote_cmd + ["exec", "print('Connected!')"], check=True, capture_output=True)
            except subprocess.CalledProcessError:
                print("Error: Could not connect to MicroPython board via serial!")
                print("Hints:")
                print("  1. Make sure the board is flashed with MicroPython (run this script with -f/--flash first).")
                print("  2. Check if the USB cable is connected to the main USB OTG port.")
                print("  3. Make sure the board is powered on.")
                sys.exit(1)
            except FileNotFoundError:
                print("Error: 'mpremote' command/module not found. Please run 'pip install mpremote' or 'pip install -r python/requirements.txt'")
                sys.exit(1)
                
            if target_script:
                print(f"[2/2] Deploying {os.path.basename(target_script)} and bootloader to the mouse...")
                
                # Copy boot.py (Hybrid Bootloader)
                boot_script = os.path.join(repo_root, "python", "boot.py")
                if os.path.exists(boot_script):
                    print("    -> Pushing boot.py (Hybrid Read-Only/Read-Write logic)...")
                    subprocess.run(mpremote_cmd + ["fs", "cp", boot_script, ":boot.py"], check=True)
                
                # Copy the target script as main.py
                print(f"    -> Pushing {os.path.basename(target_script)} as main.py...")
                subprocess.run(mpremote_cmd + ["fs", "cp", target_script, ":main.py"], check=True)
                deployed_count = 1
                if os.path.exists(boot_script):
                    deployed_count += 1
                
                # Copy other helper python files recursively from the same directory
                script_dir_path = os.path.dirname(target_script)
                
                def upload_dir_recursive(local_dir, remote_prefix=""):
                    for item in os.listdir(local_dir):
                        local_path = os.path.join(local_dir, item)
                        name_lower = item.lower()
                        if name_lower in ignore_names or name_lower in {"uct_mouse.py", "micromouse.py", "boot.py"}:
                            continue
                        _, ext = os.path.splitext(name_lower)
                        if ext in ignore_exts:
                            continue
                        if item == os.path.basename(target_script) and remote_prefix == "":
                            continue
                        
                        remote_path = f"{remote_prefix}{item}"
                        if os.path.isdir(local_path):
                            # Create remote folder
                            subprocess.run(mpremote_cmd + ["fs", "mkdir", f":{remote_path}"], capture_output=True)
                            upload_dir_recursive(local_path, f"{remote_path}/")
                        else:
                            print(f"    -> Pushing helper {remote_path}...")
                            subprocess.run(mpremote_cmd + ["fs", "cp", local_path, f":{remote_path}"], check=True)

                upload_dir_recursive(script_dir_path)
            else:
                print(f"[2/2] Mirroring {os.path.basename(target_dir)}/ development folder to the mouse...")
                
                # Push boot.py to the root first
                boot_script = os.path.join(repo_root, "python", "boot.py")
                if os.path.exists(boot_script):
                    print("    -> Pushing boot.py (Hybrid Read-Only/Read-Write logic)...")
                    subprocess.run(mpremote_cmd + ["fs", "cp", boot_script, ":boot.py"], check=True)
                
                # Copy all contents of target_dir directly to the flash root
                print(f"    -> Syncing directory contents from {target_dir} to root ...")
                for item in os.listdir(target_dir):
                    item_path = os.path.join(target_dir, item)
                    # Skip hidden files
                    if item.startswith('.'):
                        continue
                    name_lower = item.lower()
                    if name_lower in ignore_names:
                        continue
                    _, ext = os.path.splitext(name_lower)
                    if ext in ignore_exts:
                        continue
                        
                    print(f"    -> Pushing {item}...")
                    subprocess.run(mpremote_cmd + ["fs", "cp", "-r", item_path, f":{item}"], check=True)
                deployed_count = "all"
                
            print(f"Successfully deployed {deployed_count} files to the Micromouse.")
            print("Soft-rebooting the board...")
            subprocess.run(mpremote_cmd + ["soft-reset"], check=False)
            print("Done! The mouse is now running your code.")
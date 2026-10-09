import os
import sys
import shutil
import subprocess
import glob
import argparse
import time

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
            glob.glob('/Volumes/UCT_MMOUSE*') +
            glob.glob('/Volumes/UCT-MMOUSE*') +
            glob.glob('/Volumes/PYB*') +
            glob.glob('/Volumes/NO NAME*') +
            glob.glob('/Volumes/NO_NAME*')
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
                    if "UCT-MICRO" in label or "UCT_MICRO" in label or "PYB" in label or "NO NAME" in label or "NO_NAME" in label:
                        drives.append(drive_path)
            bitmask >>= 1
    else:
        drives = (
            glob.glob('/media/*/*UCT-MICROMO*') +
            glob.glob('/run/media/*/*UCT-MICROMO*') +
            glob.glob('/media/*/*UCT_MICROMO*') +
            glob.glob('/run/media/*/*UCT_MICROMO*') +
            glob.glob('/media/*/*PYB*') +
            glob.glob('/run/media/*/*PYB*') +
            glob.glob('/media/*/*NO NAME*') +
            glob.glob('/media/*/*NO_NAME*') +
            glob.glob('/run/media/*/*NO NAME*') +
            glob.glob('/run/media/*/*NO_NAME*')
        )
    return drives[0] if drives else None

def find_dfu_util_cmd():
    """Finds the dfu-util command path."""
    for cmd in ["dfu-util", "/opt/homebrew/bin/dfu-util", "/usr/local/bin/dfu-util", "/opt/local/bin/dfu-util"]:
        if shutil.which(cmd) or os.path.exists(cmd):
            return cmd
def find_st_flash_cmd():
    """Finds the st-flash command path across PATH and Windows install locations."""
    for cmd in ["st-flash", "st-flash.exe", "/opt/homebrew/bin/st-flash", "/usr/local/bin/st-flash", "/opt/local/bin/st-flash"]:
        if shutil.which(cmd) or os.path.exists(cmd):
            return cmd
    candidate_patterns = [
        r"C:\Program Files\stlink*\bin\st-flash.exe",
        r"C:\Program Files (x86)\stlink*\bin\st-flash.exe",
        r"C:\tools\stlink*\bin\st-flash.exe",
        os.path.expanduser(r"~\scoop\apps\stlink\current\bin\st-flash.exe")
    ]
    for pat in candidate_patterns:
        matches = glob.glob(pat)
        if matches and os.path.isfile(matches[0]):
            return os.path.abspath(matches[0])
    return None

def find_stm32_programmer_cli():
    """Finds STM32_Programmer_CLI (official ST-Link CLI tool on Windows/Mac/Linux)."""
    for cmd in ["STM32_Programmer_CLI", "STM32_Programmer_CLI.exe"]:
        if shutil.which(cmd) or os.path.exists(cmd):
            return cmd
    candidate_patterns = [
        r"C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",
        r"C:\Program Files (x86)\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",
        r"C:\ST\STM32CubeCLT*\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",
        r"C:\ST\STM32CubeIDE*\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.*\tools\bin\STM32_Programmer_CLI.exe",
        "/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin/STM32_Programmer_CLI"
    ]
    for pat in candidate_patterns:
        try:
            matches = glob.glob(pat, recursive=True) if "**" in pat else glob.glob(pat)
            if matches and os.path.isfile(matches[0]):
                return os.path.abspath(matches[0])
        except Exception:
            pass
def is_dfu_device_connected(dfu_util_cmd):
    """Checks if an STM32 DFU device is currently connected."""
    try:
        res = subprocess.run([dfu_util_cmd, "-l"], capture_output=True, text=True)
        return "0483:df11" in res.stdout
    except Exception:
        return False

def trigger_software_dfu_reboot():
    """Detects active MicroPython / PikaScript serial connection and issues a software reboot to DFU bootloader."""
    try:
        import serial
        import serial.tools.list_ports
        for p in serial.tools.list_ports.comports():
            # Check for MicroPython OTG or generic VCP / usbmodem port
            if (p.vid == 0xf055) or ("usbmodem" in p.device) or ("virtual com" in p.description.lower()) or ("pyboard" in p.description.lower()):
                try:
                    s = serial.Serial(p.device, 115200, timeout=0.5)
                    s.write(b'\x03\r\n')
                    time.sleep(0.05)
                    s.write(b'\r\nimport machine; machine.bootloader()\r\n')
                    time.sleep(0.05)
                    s.write(b'\r\nimport uct_mouse; uct_mouse.reboot_dfu()\r\n')
                    time.sleep(0.05)
                    s.write(b'\r\n{"c":{"dfu":1}}\r\n')
                    time.sleep(0.1)
                    s.close()
                    # Wait up to 2.5s for DFU device enumeration
                    time.sleep(1.5)
                    return True
                except Exception:
                    pass
    except Exception:
        pass
    return False

def flash_firmware(central_bin_path, erase_all=False):
    """Flashes the firmware binary onto the board.
    Tries USB DFU via dfu-util over USB OTG first (without needing ST-Link),
    then falls back to SWD via st-flash, STM32_Programmer_CLI, and ST-Link USB mass storage copy.
    If erase_all is True, performs a complete chip/mass erase before flashing to guarantee a clean slate.
    """
    dfu_util_cmd = find_dfu_util_cmd()
    if dfu_util_cmd:
        if not is_dfu_device_connected(dfu_util_cmd):
            # Check if an active firmware instance is running over USB serial and command it to enter DFU mode
            print("Checking for running firmware instance over USB OTG to enter DFU mode...")
            trigger_software_dfu_reboot()

        if is_dfu_device_connected(dfu_util_cmd):
            print(f"Using direct USB DFU flash via '{dfu_util_cmd}' (over USB OTG)...")
            try:
                dfuse_addr = "0x08000000:mass-erase:force:leave" if erase_all else "0x08000000:leave"
                subprocess.run([
                    dfu_util_cmd, 
                    "-a", "0", 
                    "-d", "0483:df11", 
                    "--dfuse-address", dfuse_addr, 
                    "-D", central_bin_path
                ], check=True)
                print("Success! Firmware flashed via USB DFU over USB OTG. Board reset triggered.")
                return True
            except (subprocess.CalledProcessError, FileNotFoundError) as e:
                print(f"Warning: USB DFU flashing failed: {e}")
                print("Falling back to SWD flashing methods...")

    # 2. Try st-flash
    st_flash_cmd = find_st_flash_cmd()
    if st_flash_cmd:
        print(f"Using direct SWD flash via '{st_flash_cmd}' (fast & reliable)...")
        try:
            if erase_all:
                print("Performing full hardware chip erase via st-flash...")
                subprocess.run([st_flash_cmd, "erase"], check=True)
            subprocess.run([st_flash_cmd, "--reset", "write", central_bin_path, "0x08000000"], check=True)
            print("Success! Firmware flashed via SWD. Board reset triggered.")
            return True
        except (subprocess.CalledProcessError, FileNotFoundError) as e:
            print(f"Warning: Direct flashing via st-flash failed: {e}")
            print("Trying STM32_Programmer_CLI...")

    # 3. Try STM32_Programmer_CLI (Official ST tool)
    stm32_cli = find_stm32_programmer_cli()
    if stm32_cli:
        print(f"Using direct SWD flash via STM32_Programmer_CLI ('{stm32_cli}')...")
        try:
            cmd = [stm32_cli, "-c", "port=SWD"]
            if erase_all:
                cmd += ["-e", "all"]
            cmd += ["-w", central_bin_path, "0x08000000", "-v", "-rst"]
            subprocess.run(cmd, check=True)
            print("Success! Firmware flashed via STM32CubeProgrammer. Board reset triggered.")
            return True
        except (subprocess.CalledProcessError, FileNotFoundError) as e:
            print(f"Warning: STM32_Programmer_CLI failed: {e}")
            print("Falling back to USB mass storage copy method...")
            
    # 4. Fallback: Find the ST-Link Mass Storage Drive
    drive = find_stlink_drive()
    if not drive:
        print("\nError: Could not find ST-Link programmer or USB DFU device.")
        print("A hardware flash/factory-reset operation requires a programmer (ST-Link) or USB DFU mode.")
        print("Please ensure the ST-Link USB cable is firmly plugged in, or enter DFU mode via BOOT0.")
        sys.exit(1)
        
    print(f"ST-Link found at {drive}. Flashing via USB mass storage stream write...")
    try:
        dest_path = os.path.join(drive, "firmware.bin")
        # Remove old firmware.bin or failed transfer artifacts if present
        for old_f in ["firmware.bin", "FAIL.TXT"]:
            old_p = os.path.join(drive, old_f)
            if os.path.exists(old_p):
                try:
                    os.remove(old_p)
                except Exception:
                    pass
                    
        # Direct raw binary stream write (avoids Windows FAT metadata allocation errors)
        with open(central_bin_path, "rb") as f_src, open(dest_path, "wb") as f_dst:
            f_dst.write(f_src.read())
            f_dst.flush()
            
        print("Success! Firmware copied to drive. Mouse will automatically reboot.")
        return True
    except Exception as e:
        print(f"Error copying to ST-Link drive: {e}")
        print("Tip: If the ST-Link virtual drive is full or locked, unplug and replug the ST-Link USB cable.")
        sys.exit(1)

def find_arm_gcc():
    """Finds arm-none-eabi-gcc executable across PATH and standard OS install directories."""
    # 1. Direct search on PATH
    for exe in ["arm-none-eabi-gcc", "arm-none-eabi-gcc.exe"]:
        p = shutil.which(exe)
        if p:
            return os.path.abspath(p)
            
    # 2. Search common installation directories
    candidate_patterns = [
        # Windows Arm GNU Toolchains (deep search)
        r"C:\Program Files (x86)\Arm GNU Toolchain*\**\arm-none-eabi-gcc.exe",
        r"C:\Program Files\Arm GNU Toolchain*\**\arm-none-eabi-gcc.exe",
        r"C:\Program Files (x86)\GNU Arm Embedded Toolchain*\**\arm-none-eabi-gcc.exe",
        r"C:\Program Files\GNU Arm Embedded Toolchain*\**\arm-none-eabi-gcc.exe",
        r"C:\Program Files (x86)\GNU Tools ARM Embedded*\**\arm-none-eabi-gcc.exe",
        r"C:\Program Files\GNU Tools ARM Embedded*\**\arm-none-eabi-gcc.exe",
        r"C:\ST\**\arm-none-eabi-gcc.exe",
        r"C:\tools\**\arm-none-eabi-gcc.exe",
        os.path.expanduser(r"~\AppData\Local\Programs\**\arm-none-eabi-gcc.exe"),
        os.path.expanduser(r"~\scoop\apps\**\arm-none-eabi-gcc.exe"),
        # MATLAB Support Packages (deep search)
        r"C:\ProgramData\MATLAB\SupportPackages\**\arm-none-eabi-gcc.exe",
        os.path.expanduser(r"~\AppData\Roaming\MathWorks\**\arm-none-eabi-gcc.exe"),
        r"C:\Program Files\MATLAB\**\arm-none-eabi-gcc.exe",
        # Unix/macOS paths
        "/opt/homebrew/bin/arm-none-eabi-gcc",
        "/usr/local/bin/arm-none-eabi-gcc",
        "/opt/local/bin/arm-none-eabi-gcc",
        "/usr/bin/arm-none-eabi-gcc"
    ]
    
    for pat in candidate_patterns:
        try:
            matches = glob.glob(pat, recursive=True) if "**" in pat else glob.glob(pat)
            if matches:
                # Ensure the matched file is executable / valid
                for m in matches:
                    if os.path.isfile(m):
                        return os.path.abspath(m)
        except Exception:
            pass
            
    return None

def get_cmake_toolchain_flags(required=True):
    """Generates CMake toolchain compiler arguments and ensures arm-none-eabi-gcc is in PATH."""
    arm_gcc = find_arm_gcc()
    if not arm_gcc:
        if required:
            print("\nError: ARM GCC cross-compiler ('arm-none-eabi-gcc') not found on your system.")
            print("To compile Simulink or C firmware, please install the ARM GNU toolchain:")
            print("  Windows (PowerShell): winget install Arm.GnuArmEmbeddedToolchain")
            print("       or (Chocolatey): choco install make arm-none-eabi-gcc")
            print("  macOS:                brew install arm-none-eabi-gcc")
            print("  Linux (Ubuntu/Debian): sudo apt install gcc-arm-none-eabi")
            sys.exit(1)
        else:
            return None
        
    gcc_dir = os.path.dirname(arm_gcc)
    if gcc_dir not in os.environ.get("PATH", ""):
        os.environ["PATH"] = gcc_dir + os.pathsep + os.environ.get("PATH", "")
        
    exe_ext = ".exe" if sys.platform.startswith("win") else ""
    gxx = os.path.join(gcc_dir, f"arm-none-eabi-g++{exe_ext}")
    objcopy = os.path.join(gcc_dir, f"arm-none-eabi-objcopy{exe_ext}")
    size = os.path.join(gcc_dir, f"arm-none-eabi-size{exe_ext}")
    
    flags = [
        f"-DCMAKE_C_COMPILER={arm_gcc.replace(os.sep, '/')}",
        f"-DCMAKE_ASM_COMPILER={arm_gcc.replace(os.sep, '/')}"
    ]
    if os.path.exists(gxx):
        flags.append(f"-DCMAKE_CXX_COMPILER={gxx.replace(os.sep, '/')}")
    if os.path.exists(objcopy):
        flags.append(f"-DCMAKE_OBJCOPY={objcopy.replace(os.sep, '/')}")
    if os.path.exists(size):
        flags.append(f"-DCMAKE_SIZE={size.replace(os.sep, '/')}")
        
    if sys.platform.startswith("win"):
        if shutil.which("ninja"):
            flags += ["-G", "Ninja"]
        elif shutil.which("mingw32-make") or shutil.which("make"):
            flags += ["-G", "MinGW Makefiles"]
            
    return flags

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
    parser.add_argument(
        "--format-drive",
        action="store_true",
        help="Format the virtual USB storage partition (UCT_MMOUSE) and restore default boot.py and main.py over USB OTG without hardware reflashing."
    )
    parser.add_argument(
        "--factory-reset",
        action="store_true",
        help="Perform a true hardware factory reset: completely erase the microcontroller flash (all firmware, filesystem, and telemetry partitions) and reflash pristine base firmware from scratch using ST-Link or USB DFU."
    )
    args = parser.parse_args()

    print("=== UCT Micromouse Firmware Deployer ===")
    print(f"Selected engine: {args.engine.upper()}")
    
    # Dynamically resolve paths so the script can be run from anywhere
    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.abspath(os.path.join(script_dir, ".."))

    # Define files/folders to ignore during deployment
    ignore_names = {
        "deploy", "ekf_research", "attic", "build", "matlab", "external", 
        "tools", ".git", "__pycache__", "sub1_marking", "sub2_marking", 
        "ga1_marking", "ga3_marking", "marking", "extracted_pages", "extracted_text",
        "raw_submissions", "tmp", "venv", ".venv", "env", ".vscode", ".idea"
    }
    ignore_exts = {
        ".pdf", ".zip", ".docx", ".md", ".slx", ".slxc", ".bin", ".elf", 
        ".map", ".mp4", ".png", ".jpg", ".jpeg", ".bmp", ".gif", ".svg",
        ".csv", ".tsv", ".tar", ".gz", ".7z", ".pkl", ".mat", ".log", ".jsonl"
    }

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
            toolchain_flags = get_cmake_toolchain_flags()
            subprocess.run(["cmake", "-S", "firmware", "-B", "firmware/build"] + toolchain_flags, cwd=repo_root, check=True)
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
        print("[1/2] Preparing Simulink firmware binary...")
        
        # 1. Search for generated Simulink code directory (*_ert_rtw)
        ert_candidates = []
        search_roots = [
            os.path.join(repo_root, "build"),
            os.path.join(repo_root, "matlab", "simulink"),
            os.path.join(repo_root, "matlab"),
            os.path.join(repo_root, "workspace"),
            repo_root
        ]
        
        for root_path in search_roots:
            if os.path.exists(root_path):
                for entry in os.listdir(root_path):
                    full_p = os.path.join(root_path, entry)
                    if entry.endswith("_ert_rtw") and os.path.isdir(full_p):
                        # Verify it contains C sources
                        c_files = [f for f in os.listdir(full_p) if f.endswith(".c") and f != "ert_main.c"]
                        if c_files:
                            mtime = os.path.getmtime(full_p)
                            ert_candidates.append((mtime, full_p, entry))
                            
        selected_ert_dir = None
        if ert_candidates:
            # Sort by newest modification time
            ert_candidates.sort(key=lambda x: x[0], reverse=True)
            selected_ert_dir = ert_candidates[0][1]
            model_name = ert_candidates[0][2].replace("_ert_rtw", "")
            print(f"    -> Found generated Simulink model code: {selected_ert_dir} ({model_name})")
            
        untracked_bin_path = os.path.join(repo_root, "build", "bin", "simulink.bin")
        tracked_bin_path = os.path.join(repo_root, "firmware", "binaries", "simulink.bin")
        active_bin_path = None
        
        toolchain_flags = get_cmake_toolchain_flags(required=False)
        
        if selected_ert_dir and toolchain_flags is not None:
            try:
                print("    -> Configuring CMake for Simulink model...")
                ert_arg = f"-DSIMULINK_ERT_DIR={selected_ert_dir.replace(os.sep, '/')}"
                subprocess.run(
                    ["cmake", "-S", "firmware", "-B", "firmware/build", ert_arg] + toolchain_flags,
                    cwd=repo_root,
                    check=True
                )
                print("    -> Building Simulink firmware target (ARM cross-compiler)...")
                subprocess.run(
                    ["cmake", "--build", "firmware/build", "--target", "simulink_firmware"],
                    cwd=repo_root,
                    check=True
                )
                built_bin = os.path.join(repo_root, "firmware", "build", "simulink_firmware.bin")
                if os.path.exists(built_bin):
                    os.makedirs(os.path.dirname(untracked_bin_path), exist_ok=True)
                    shutil.copy(built_bin, untracked_bin_path)
                    shutil.copy(built_bin, tracked_bin_path)
                    active_bin_path = untracked_bin_path
                    print(f"    -> Successfully compiled fresh binary to {untracked_bin_path}")
            except subprocess.CalledProcessError as e:
                print(f"    -> Note: CMake build encountered: {e}")

        # 1. Check if MATLAB generated a .bin file directly inside the ert_rtw folder
        if selected_ert_dir and not active_bin_path:
            for f in os.listdir(selected_ert_dir):
                if f.endswith(".bin"):
                    direct_bin = os.path.join(selected_ert_dir, f)
                    print(f"    -> Found direct binary in model folder: {direct_bin}")
                    active_bin_path = direct_bin
                    break

        if not active_bin_path:
            if os.path.exists(untracked_bin_path):
                print(f"    -> Flashing compiled model binary: {untracked_bin_path}")
                active_bin_path = untracked_bin_path
            elif os.path.exists(tracked_bin_path):
                print(f"    -> Flashing firmware release binary: {tracked_bin_path}")
                active_bin_path = tracked_bin_path
            else:
                print("Error: No Simulink firmware binary found.")
                print("Please build your Simulink model in MATLAB (Cmd+B) or install arm-none-eabi-gcc.")
                sys.exit(1)

        # Flash the compiled firmware onto the board
        print(f"[2/2] Flashing Simulink firmware...")
        flash_firmware(active_bin_path)
        print("Success! Simulink firmware is flashed. The board will automatically reboot and execute.")

    else:
        # === MICROPYTHON ENGINE FLOW ===
        if args.flash or getattr(args, 'factory_reset', False):
            # --- Flashing the MicroPython C-Firmware / Factory Reset ---
            action_desc = "Factory Reset (Chip Erase & Reflash)" if args.factory_reset else "Flashing MicroPython firmware"
            print(f"[1/2] Preparing MicroPython firmware binary for {action_desc}...")
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
            
            # Flash the compiled firmware onto the board (with full chip erase if factory reset)
            print(f"[2/2] {action_desc}...")
            flash_firmware(active_bin_path, erase_all=args.factory_reset)
            if args.factory_reset:
                print("\n========================================================================")
                print("[Factory Reset Complete]")
                print("Microcontroller flash has been completely erased and reflashed from baseline firmware.")
                print("All persistent storage and telemetry partitions have been wiped clean.")
                print("On boot, the board will initialize with clean out-of-the-box factory defaults.")
                print("========================================================================\n")
                sys.exit(0)
            else:
                print("Success! MicroPython interpreter is flashed. The board will reboot and mount as a USB drive shortly.")
            
        else:
            # --- Deploying Python Scripts via VCP (mpremote) / USB Drive ---
            print("[1/2] Connecting to MicroPython via Serial...")
            
            # Dynamically detect candidate serial ports (MicroPython USB OTG and ST-Link VCP)
            mpy_port = None
            stlink_port = None
            try:
                import serial.tools.list_ports
                for p in serial.tools.list_ports.comports():
                    if p.vid == 0xf055 and p.pid in (0x9800, 0x9801, 0x9802):
                        mpy_port = p.device
                    elif "ST-Link" in p.description or "STLink" in p.description or (p.vid == 0x0483 and p.pid in (0x374b, 0x3752)):
                        stlink_port = p.device
                    elif "usbmodem" in p.device:
                        if "Pyboard" in p.description or "Virtual Comm Port" in p.description:
                            mpy_port = p.device
                        elif not stlink_port:
                            stlink_port = p.device
            except Exception:
                pass
                
            active_port = mpy_port if mpy_port else stlink_port
            mpremote_cmd = [sys.executable, "-m", "mpremote"]
            if active_port:
                port_type = "USB OTG" if active_port == mpy_port else "ST-Link VCP"
                print(f"    -> Detected MicroPython on {active_port} ({port_type})")
                # Send raw interrupt to free REPL if busy
                try:
                    import serial
                    s_int = serial.Serial(active_port, 115200, timeout=0.2)
                    s_int.write(b'\x03\x03')
                    time.sleep(0.1)
                    s_int.close()
                except Exception:
                    pass
                mpremote_cmd += ["connect", active_port]
            
            mpy_drive = find_micropython_drive()
            use_direct_copy = False
            
            if getattr(args, 'format_drive', False):
                print("[2/2] Formatting virtual USB flash filesystem (UCT_MMOUSE)...")
                format_script = (
                    "import os, pyb\n"
                    "try:\n"
                    "    os.umount('/flash')\n"
                    "except Exception:\n"
                    "    pass\n"
                    "f = pyb.Flash()\n"
                    "print('Formatting FAT partition...')\n"
                    "os.VfsFat.mkfs(f)\n"
                    "vfs = os.VfsFat(f)\n"
                    "os.mount(vfs, '/flash')\n"
                    "with open('/flash/boot.py', 'w') as fp:\n"
                    "    fp.write('# boot.py - UCT Micromouse Bootloader\\n'\n"
                    "             'import os, pyb\\n\\n'\n"
                    "             '# Auto-purge macOS metadata bloat (._* and .DS_Store)\\n'\n"
                    "             'try:\\n'\n"
                    "             '    for f in os.listdir(\"/flash\"):\\n'\n"
                    "             '        if f.startswith(\"._\") or f in (\".DS_Store\", \".Trashes\"):\\n'\n"
                    "             '            try: os.remove(\"/flash/\" + f)\\n'\n"
                    "             '            except Exception: pass\\n'\n"
                    "             'except Exception:\\n'\n"
                    "             '    pass\\n\\n'\n"
                    "             'pyb.main(\"main.py\")\\n')\n"
                    "with open('/flash/main.py', 'w') as fp:\n"
                    "    fp.write('# main.py -- UCT Micromouse Default Telemetry Streamer\\n'\n"
                    "             'import uct_mouse\\n\\n'\n"
                    "             '# Initialize hardware (enables OLED display and sensor polling)\\n'\n"
                    "             'uct_mouse.init()\\n'\n"
                    "             'uct_mouse.set_motors(0, 0)\\n\\n'\n"
                    "             'print(\"--- UCT Micromouse Online ---\")\\n'\n"
                    "             'print(\"Streaming live telemetry. Replace main.py with your code!\")\\n\\n'\n"
                    "             'while True:\\n'\n"
                    "             '    tof = uct_mouse.get_tof()\\n'\n"
                    "             '    enc = uct_mouse.get_encoders()\\n'\n"
                    "             '    vbatt = uct_mouse.get_vbatt()\\n'\n"
                    "             '    gyro = uct_mouse.get_gyro()\\n'\n"
                    "             '    print(\"VBatt: %.2fV | Gyro: %+.2f dps | Enc: (%d, %d) | ToF: %s\" % (vbatt, gyro, enc[0], enc[1], str(tof)))\\n'\n"
                    "             '    uct_mouse.delay_ms(250)\\n')\n"
                    "with open('/flash/README.txt', 'w') as fp:\n"
                    "    fp.write('UCT Micromouse MicroPython Drive (STM32L476VE)\\n')\n"
                    "with open('/flash/.metadata_never_index', 'w') as fp:\n"
                    "    pass\n"
                    "print('Flash formatted and mounted successfully!')\n"
                )
                reset_done = False
                candidate_ports = [p for p in [mpy_port, stlink_port] if p]
                for cand in candidate_ports:
                    try:
                        import serial
                        time.sleep(0.2)
                        s = serial.Serial(cand, 115200, timeout=2.5)
                        s.write(b'\r\x03\x03')
                        time.sleep(0.1)
                        s.read_all()
                        s.write(b'\r\x01')
                        time.sleep(0.1)
                        s.read_until(b'>')
                        s.write(format_script.encode('utf-8') + b'\x04')
                        time.sleep(0.5)
                        s.read_until(b'OK')
                        out = s.read_until(b'\x04')
                        err = s.read_until(b'>')
                        if out:
                            print(out.decode('utf-8', errors='replace').strip())
                        s.write(b'\r\x04') # soft reset
                        time.sleep(0.2)
                        s.close()
                        reset_done = True
                        print(f"Drive format executed successfully via {cand}.")
                        break
                    except Exception as e:
                        print(f"Serial format attempt on {cand} encountered: {e}")

                if not reset_done:
                    try:
                        subprocess.run(mpremote_cmd + ["exec", format_script], check=True)
                        subprocess.run(mpremote_cmd + ["soft-reset"], check=False)
                        reset_done = True
                    except Exception:
                        pass

                if reset_done:
                    print("\n[Drive Format Complete]")
                    print("The virtual USB storage partition (/Volumes/UCT_MMOUSE) has been freshly formatted.")
                    print("Default boot.py and main.py have been restored.")
                    sys.exit(0)
                else:
                    print("Error: Could not format drive via active serial connection.")
                    sys.exit(1)

            try:
                subprocess.run(mpremote_cmd + ["exec", "print('Connected!')"], check=True, capture_output=True)
            except (subprocess.CalledProcessError, FileNotFoundError):
                if mpy_drive:
                    print(f"Serial connection busy or mpremote failed. Falling back to direct filesystem copy to: {mpy_drive}")
                    use_direct_copy = True
                else:
                    print("Error: Could not connect to MicroPython board via serial!")
                    print("Hints:")
                    print("  1. Make sure the board is flashed with MicroPython (run this script with -f/--flash first).")
                    print("  2. Check if the USB cable is connected to the main USB OTG port or ST-Link.")
                    print("  3. Make sure the board is powered on.")
                    sys.exit(1)

            if target_script:
                print(f"[2/2] Deploying {os.path.basename(target_script)} and bootloader to the mouse...")
                
                # Copy boot.py (Hybrid Bootloader)
                boot_script = os.path.join(repo_root, "python", "boot.py")
                if os.path.exists(boot_script):
                    print("    -> Pushing boot.py (Hybrid Read-Only/Read-Write logic)...")
                    if use_direct_copy:
                        shutil.copyfile(boot_script, os.path.join(mpy_drive, "boot.py"))
                    else:
                        subprocess.run(mpremote_cmd + ["fs", "cp", boot_script, ":boot.py"], check=True)
                
                # Copy the target script as main.py
                print(f"    -> Pushing {os.path.basename(target_script)} as main.py...")
                if use_direct_copy:
                    shutil.copyfile(target_script, os.path.join(mpy_drive, "main.py"))
                else:
                    subprocess.run(mpremote_cmd + ["fs", "cp", target_script, ":main.py"], check=True)
                deployed_count = 1
                if os.path.exists(boot_script):
                    deployed_count += 1
                

            else:
                print(f"[2/2] Mirroring {os.path.basename(target_dir)}/ development folder to the mouse...")
                
                # Push boot.py to the root first
                boot_script = os.path.join(repo_root, "python", "boot.py")
                if os.path.exists(boot_script):
                    print("    -> Pushing boot.py (Hybrid Read-Only/Read-Write logic)...")
                    if use_direct_copy:
                        shutil.copyfile(boot_script, os.path.join(mpy_drive, "boot.py"))
                    else:
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
                    if use_direct_copy:
                        if os.path.isdir(item_path):
                            dest_path = os.path.join(mpy_drive, item)
                            if os.path.exists(dest_path):
                                shutil.rmtree(dest_path)
                            shutil.copytree(item_path, dest_path)
                        else:
                            shutil.copy2(item_path, os.path.join(mpy_drive, item))
                    else:
                        subprocess.run(mpremote_cmd + ["fs", "cp", "-r", item_path, f":{item}"], check=True)
                deployed_count = "all"
                
            if use_direct_copy and mpy_drive and os.path.exists(mpy_drive):
                # Clean up any macOS AppleDouble metadata files to preserve FAT storage space
                for root, dirs, files in os.walk(mpy_drive):
                    for f in files:
                        if f.startswith("._"):
                            try:
                                os.remove(os.path.join(root, f))
                            except Exception:
                                pass

            print(f"Successfully deployed {deployed_count} files to the Micromouse.")
            if not use_direct_copy:
                print("Soft-rebooting the board...")
                subprocess.run(mpremote_cmd + ["soft-reset"], check=False)
            print("Done! The mouse is now running your code.")
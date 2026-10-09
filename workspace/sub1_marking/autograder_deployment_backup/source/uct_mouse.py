# =========================================================================
# UCT Micromouse - Tier 2 Python PC Mock Wrapper
# =========================================================================
# This module perfectly mimics the hardware PikaScript uct_mouse API.
# When students run main.py on their PC, this silently connects to the 
# Simulink TCP Virtual Testbed instead of physical hardware!
# =========================================================================

import time
import os
import sys
import json
import socket
import subprocess
import atexit
from micromouse import Micromouse

# Resolve configuration file paths
_script_dir = os.path.dirname(os.path.abspath(__file__))
_config_path = os.path.join(_script_dir, "sim_config.json")

# Track the auto-started simulator process
_backend_process = None

def _is_port_active(host="127.0.0.1", port=8000):
    """Checks if a TCP port is active (listening) on localhost."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(0.1)
    try:
        s.connect((host, port))
        s.close()
        return True
    except Exception:
        return False

def _cleanup_backend():
    """Terminate the auto-started simulator process on exit."""
    global _backend_process
    if _backend_process is not None:
        try:
            _backend_process.terminate()
            _backend_process.wait(timeout=1.0)
        except Exception:
            try:
                _backend_process.kill()
            except Exception:
                pass
        _backend_process = None

_target_host = os.environ.get("UCT_MICROMOUSE_HOST", "127.0.0.1")
_target_port = int(os.environ.get("UCT_MICROMOUSE_PORT", "8000"))

# Create a global background TCP instance
_mouse = Micromouse(method='tcp', host=_target_host, port=_target_port, verbose=False)
_pending_pwm_l = 0
_pending_pwm_r = 0
_pwm_dirty = False

# Keep track of original sleep for restoring/using inside delay_ms
_original_sleep = time.sleep

# Virtual simulation clock tracking in milliseconds
_virtual_time_ms = 0

def _ticks_ms():
    """Returns simulated virtual time in milliseconds (commensurate with hardware HAL_GetTick)."""
    return int(_virtual_time_ms)

def _ticks_us():
    """Returns simulated virtual time in microseconds."""
    return int(_virtual_time_ms * 1000)

def _ticks_cpu():
    """Returns simulated virtual CPU ticks."""
    return int(_virtual_time_ms * 1000)

def _ticks_diff(t1, t2):
    """Computes signed difference between two tick values with standard 30-bit wraparound handling."""
    diff = (int(t1) - int(t2)) & 0x3FFFFFFF
    if diff & 0x20000000:
        diff -= 0x40000000
    return diff

def _ticks_add(ticks, delta):
    """Adds a delta in milliseconds to a tick value."""
    return (int(ticks) + int(delta)) & 0x3FFFFFFF

def _sleep_ms(ms):
    """Paces simulation physics by ms milliseconds."""
    delay_ms(ms)

def _sleep_us(us):
    """Paces simulation physics by us microseconds."""
    delay_ms(max(1, int(us / 1000)))

# Attach MicroPython time extensions onto standard Python time module for desktop compatibility
for _attr_name, _attr_fn in [
    ("ticks_ms", _ticks_ms),
    ("ticks_us", _ticks_us),
    ("ticks_cpu", _ticks_cpu),
    ("ticks_diff", _ticks_diff),
    ("ticks_add", _ticks_add),
    ("sleep_ms", _sleep_ms),
    ("sleep_us", _sleep_us),
]:
    if not hasattr(time, _attr_name):
        setattr(time, _attr_name, _attr_fn)

# Fast simulation tracking variable
_is_fast_sim_active = (
    os.environ.get("GRADESCOPE_AUTOGRADER") == "1" or
    os.environ.get("UCT_MICROMOUSE_FAST_SIM") == "1" or
    os.environ.get("UCT_OFFLINE_MODE") == "1"
)

def _fast_sleep(seconds):
    delay_ms(int(seconds * 1000))

def _apply_sleep_interceptor():
    time.sleep = _fast_sleep
    try:
        frame = sys._getframe()
        while frame:
            if "sleep" in frame.f_globals and frame.f_globals["sleep"] is not _fast_sleep:
                frame.f_globals["sleep"] = _fast_sleep
            frame = frame.f_back
    except Exception:
        pass

def _restore_sleep_interceptor():
    time.sleep = _original_sleep
    try:
        frame = sys._getframe()
        while frame:
            if "sleep" in frame.f_globals and frame.f_globals["sleep"] is _fast_sleep:
                frame.f_globals["sleep"] = _original_sleep
            frame = frame.f_back
    except Exception:
        pass

def set_fast_sim(enable):
    """Programmatically enable or disable fast simulation mode."""
    global _is_fast_sim_active
    _is_fast_sim_active = bool(enable)
    if _is_fast_sim_active:
        _apply_sleep_interceptor()
    else:
        _restore_sleep_interceptor()

# Apply the interceptor at import-time if the environment variables specify it
if _is_fast_sim_active:
    _apply_sleep_interceptor()

def init(fast_sim=None):
    """Connects to the configured simulation backend, auto-starting if enabled.
    
    Args:
        fast_sim (bool, optional): Overrides the fast simulation setting. If True, running in high-speed,
                                  physics-stepped offline mode bypassing standard time sleep delays.
                                  If False, fast simulation mode is disabled.
                                  If None, falls back to config file setting or environment variables.
    """
    global _backend_process, _is_fast_sim_active
    
    # Default configuration
    config = {
        "backend": "simulink",
        "auto_start": False,
        "map": "empty",
        "imbalance": 0.08,
        "slip": 0.08,
        "video": "",
        "fast_sim": None
    }
    
    # Load local config overrides if present
    if os.path.exists(_config_path):
        try:
            with open(_config_path, "r") as f:
                loaded = json.load(f)
                config.update(loaded)
        except Exception as e:
            print(f"[PC Mock] Warning: Failed to read {os.path.basename(_config_path)}: {e}")
            
    # Determine fast_sim status: code parameter takes precedence, then json config, then existing _is_fast_sim_active (env vars)
    if fast_sim is not None:
        _is_fast_sim_active = bool(fast_sim)
    elif config.get("fast_sim") is not None:
        _is_fast_sim_active = bool(config["fast_sim"])
        
    if _is_fast_sim_active:
        config["auto_start"] = False
        _apply_sleep_interceptor()
    else:
        _restore_sleep_interceptor()

    backend = config.get("backend", "simulink").lower()
    auto_start = config.get("auto_start", False)
    
    # Attempt to connect to an already running simulator on target port
    connected_sock = None
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.2)
        s.connect((_target_host, _target_port))
        s.settimeout(None)
        connected_sock = s
    except Exception:
        pass  # No server running, or connection timed out/refused
        
    auto_started = False
    if connected_sock is None:
        if backend == "python" and auto_start:
            sim_script = os.path.join(os.path.dirname(_script_dir), "tools", "physics_sim.py")
            if os.path.exists(sim_script):
                # Construct physics_sim command line arguments from config
                cmd = [sys.executable, sim_script, "--port", str(_target_port)]
                if "map" in config:
                    cmd += ["--map", str(config["map"])]
                if "seed" in config and config["seed"] is not None:
                    cmd += ["--seed", str(config["seed"])]
                if "imbalance" in config:
                    cmd += ["--imbalance", str(config["imbalance"])]
                if "slip" in config:
                    cmd += ["--slip", str(config["slip"])]
                if "video" in config and config["video"]:
                    cmd += ["--video", str(config["video"])]
                    
                print(f"[PC Mock] Auto-starting Python Physics Simulator...")
                try:
                    # Spawn the simulator process
                    _backend_process = subprocess.Popen(
                        cmd,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL
                    )
                    # Register exit handler to clean up when main process exits
                    atexit.register(_cleanup_backend)
                    
                    # Poll target port until active
                    for _ in range(30):  # Wait up to 1.5 seconds
                        time.sleep(0.05)
                        try:
                            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                            s.settimeout(0.1)
                            s.connect((_target_host, _target_port))
                            s.settimeout(None)
                            connected_sock = s
                            auto_started = True
                            break
                        except Exception:
                            pass
                    if connected_sock is None:
                        print(f"[PC Mock] Error: Python simulator started but port {_target_port} did not become active.")
                except Exception as e:
                    print(f"[PC Mock] Error spawning simulator: {e}")
            else:
                print(f"[PC Mock] Error: Simulator script not found at {sim_script}")
        else:
            if backend == "simulink":
                print("[PC Mock] Waiting for Simulink Virtual Testbed...")
                print("[PC Mock] Please open MATLAB and run matlab/simulink/launch_virtual_testbed.m")
                # Fall back to standard connect which has a retry loop and prints messages
                try:
                    _mouse.connect()
                    # Lock simulation physics to 100Hz (0.01s steps) to match Python loop
                    _mouse.configure(rate=100, sync=1)
                    print("[PC Mock] Connected to Simulink Virtual Testbed.")
                    return True
                except Exception as e:
                    print(f"[PC Mock] Connection failed: {e}")
                    print("[PC Mock] Hint: Make sure Simulink Virtual Testbed is running in MATLAB.")
                    return False
            else:
                print("[PC Mock] No active server on port 8000. Ensure simulator is running.")
                return False
                
    if connected_sock is not None:
        try:
            # Bind the successfully connected socket to the Micromouse instance
            _mouse.sock = connected_sock
            _mouse.connected = True
            _mouse.configure(rate=100, sync=1)
            
            if backend == "python":
                if auto_started:
                    print("[PC Mock] Connected to Python Physics Simulator (Auto-started).")
                else:
                    print("[PC Mock] Connected to Python Physics Simulator (Pre-running).")
            else:
                print("[PC Mock] Connected to Simulink Virtual Testbed.")
            return True
        except Exception as e:
            print(f"[PC Mock] Connection initialization failed: {e}")
            return False
    else:
        return False

def set_motors(left_pwm, right_pwm):
    """Sends motor commands to the virtual physics engine without instantly advancing time."""
    global _pending_pwm_l, _pending_pwm_r, _pwm_dirty
    _pending_pwm_l = int(left_pwm)
    _pending_pwm_r = int(right_pwm)
    _pwm_dirty = True
    # We do NOT immediately exchange data here. delay_ms will pace the simulation.

def get_tof():
    """Returns (left, front_left, center, front_right, right) filtered ToF distances in mm (8190 if out-of-range)."""
    s = _mouse.get_sensors()
    return s.get('tof_l', 8190), s.get('tof_al', 8190), s.get('tof_c', 8190), s.get('tof_ar', 8190), s.get('tof_r', 8190)

def get_tof_raw():
    """Returns (left, front_left, center, front_right, right) raw un-thresholded ToF distances in mm."""
    s = _mouse.get_sensors()
    return s.get('tof_raw_l', s.get('tof_l', 8190)), s.get('tof_raw_al', s.get('tof_al', 8190)), \
           s.get('tof_raw_c', s.get('tof_c', 8190)), s.get('tof_raw_ar', s.get('tof_ar', 8190)), \
           s.get('tof_raw_r', s.get('tof_r', 8190))

def _calc_sim_signal(dist_mm):
    if dist_mm <= 0 or dist_mm >= 2000:
        return 0
    # Realistic inverse-square photon return rate in kcps (approx 800 kcps @ 150mm, 200 kcps @ 300mm)
    return max(10, min(2000, int(18000000 / (dist_mm * dist_mm + 100))))

def get_tof_signals():
    """Returns (left, front_left, center, front_right, right) return signal rates in kcps."""
    s = _mouse.get_sensors()
    if 'tof_sig_l' in s:
        return s['tof_sig_l'], s['tof_sig_al'], s['tof_sig_c'], s['tof_sig_ar'], s['tof_sig_r']
    dists = get_tof_raw()
    return tuple(_calc_sim_signal(d) for d in dists)

def get_tof_detailed():
    """Returns tuple of 5 pairs: ((left_dist, left_sig), (front_left_dist, front_left_sig), ...) in mm and kcps."""
    dists = get_tof_raw()
    sigs = get_tof_signals()
    return tuple((dists[i], sigs[i]) for i in range(5))

def get_gyro():
    """Returns virtual gyro reading (yaw rate or angle depending on context, typically deg/s or relative heading)."""
    s = _mouse.get_sensors()
    return s.get('gyro', 0.0)

_left_enc_polarity = 1
_right_enc_polarity = 1

def get_encoders():
    """Returns (left, right) virtual encoder ticks."""
    s = _mouse.get_sensors()
    return s.get('lenc', 0) * _left_enc_polarity, s.get('renc', 0) * _right_enc_polarity

def get_vbatt():
    """Returns virtual battery voltage."""
    s = _mouse.get_sensors()
    return s.get('v_batt', 0.0)

def delay_ms(ms):
    """Pauses the Python thread while advancing simulator physics in lock-step."""
    global _pwm_dirty, _virtual_time_ms
    ms = int(ms)
    _virtual_time_ms += ms
    
    rate_hz = 100  # Sync rate: 100Hz (10ms steps)
    step_ms = 1000.0 / rate_hz
    steps = int(ms / step_ms)
    
    if steps <= 0:
        if not _is_fast_sim_active:
            _original_sleep(ms / 1000.0)
        return
        
    for _ in range(steps):
        if _mouse.connected:
            try:
                if _pwm_dirty:
                    _mouse.set_pwm(_pending_pwm_l, _pending_pwm_r)
                    _pwm_dirty = False
                else:
                    _mouse.poll()
            except Exception:
                pass
        if not _is_fast_sim_active:
            _original_sleep(step_ms / 1000.0)
        
    # Sleep any remaining fractional time
    rem_ms = ms % step_ms
    if rem_ms > 0 and not _is_fast_sim_active:
        _original_sleep(rem_ms / 1000.0)

def set_polarity(left_polarity, right_polarity):
    """Sets motor polarity multipliers on the physical mouse (ignored on PC)."""
    pass

def set_encoder_polarity(left_polarity, right_polarity):
    """Sets encoder polarity multipliers on PC mock or physical mouse."""
    global _left_enc_polarity, _right_enc_polarity
    _left_enc_polarity = left_polarity
    _right_enc_polarity = right_polarity

def get_line_sensors():
    """Returns (front_left, front_right, side_left, side_right) downward facing line sensors."""
    s = _mouse.get_sensors()
    return s.get('ir_fl', 0), s.get('ir_fr', 0), s.get('ir_sl', 0), s.get('ir_sr', 0)

def get_button():
    """Returns the state of the user button SW1 (btn1)."""
    s = _mouse.get_sensors()
    return s.get('btn1', 0)

def log_custom(json_str):
    """Logs a custom JSON string (prints to stdout on PC; writes to flash on mouse)."""
    print(f"[LOG_CUSTOM] {json_str}")

def get_ticks_ms():
    """Returns elapsed time in milliseconds (commensurate with hardware HAL_GetTick)."""
    return int(_virtual_time_ms)

# Export aliases for MicroPython time module compatibility directly on uct_mouse
ticks_ms = _ticks_ms
ticks_us = _ticks_us
ticks_cpu = _ticks_cpu
ticks_diff = _ticks_diff
ticks_add = _ticks_add
sleep_ms = _sleep_ms
sleep_us = _sleep_us

def dump_logs():
    """Triggers telemetry log dump over VCP (ignored on PC)."""
    pass

def erase_flash():
    """Triggers complete external SPI flash erase on physical mouse (ignored on PC)."""
    pass

def display_text(row, text):
    """Writes custom text to OLED display row 1..4 (or None/empty string to restore default telemetry)."""
    if text:
        print(f"[OLED Line {row}] {text}")
    else:
        print(f"[OLED Line {row}] <Default Telemetry Restored>")

def set_display_text(row, text):
    """Alias for display_text(row, text)."""
    display_text(row, text)

def clear_display():
    """Restores all custom OLED rows 1..4 back to default telemetry."""
    print("[OLED] All rows restored to default telemetry.")
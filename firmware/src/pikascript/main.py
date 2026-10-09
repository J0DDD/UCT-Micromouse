# =========================================================================
# UCT Micromouse - Live Multi-Sensor & ToF Signal Diagnostics
# =========================================================================
# Demonstrates:
#   1. uct_mouse.get_tof()         -> Default reliable filtered distances (mm)
#   2. uct_mouse.get_tof_raw()     -> Unfiltered raw distances (mm)
#   3. uct_mouse.get_tof_signals() -> Return signal strength in kcps
#   4. uct_mouse.display_text()    -> Live multi-line OLED telemetry
# =========================================================================

import uct_mouse
import time

def main():
    if not uct_mouse.init():
        print("Error: uct_mouse.init() failed.")
        return

    print("==================================================")
    print("  UCT Micromouse: Live ToF & Signal Diagnostics   ")
    print("==================================================")
    print("Format: [Sensor] Filtered(mm) | Raw(mm) | Signal(kcps)")

    count = 0
    while True:
        # 1. Read all three ToF telemetry streams
        tof_filt = uct_mouse.get_tof()
        tof_raw  = uct_mouse.get_tof_raw()
        tof_sig  = uct_mouse.get_tof_signals()
        
        # 2. Extract Center, Left, Right
        l_f, fl_f, c_f, fr_f, r_f = tof_filt
        l_r, fl_r, c_r, fr_r, r_r = tof_raw
        l_s, fl_s, c_s, fr_s, r_s = tof_sig
        
        # 3. Print periodically to Serial REPL
        if count % 5 == 0:
            print(f"C: {c_f:4d} mm (raw:{c_r:4d}, sig:{c_s:4d} kcps) | "
                  f"L: {l_f:4d} (sig:{l_s:3d}) | R: {r_f:4d} (sig:{r_s:3d})")

        # 4. Update OLED Display
        # Row 0 is kept as default ('MicroPython')
        # Rows 1-3 display live readings
        uct_mouse.display_text(1, f"C: {c_f}mm ({c_s}k)")
        uct_mouse.display_text(2, f"L:{l_f} R:{r_f}")
        uct_mouse.display_text(3, f"Raw C:{c_r}mm")
        
        uct_mouse.delay_ms(100)
        count += 1

if __name__ == "__main__":
    main()

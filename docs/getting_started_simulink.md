# Quickstart Guide: Simulink Track

This guide will take you from setting up your MATLAB path to compiling and flashing your first Simulink controller to the physical mouse.

---

## 🛠️ Step 1: Initialize the Workspace

Before running any Simulink models, you must initialize the search paths and setup directories:

1. Launch **MATLAB** (2024b or newer recommended).
2. Change your current directory to the **`matlab/`** folder in this repository.
3. Run the **`startup.m`** script:
   ```matlab
   run startup.m
   ```
4. This adds all nested simulation and model directories to MATLAB's search path and creates the build folders.

---

## 💻 Step 2: Run a Desktop Co-Simulation (No Hardware Needed)

Test your algorithm in the Pygame visual maze simulator:

1. Open `matlab/simulink/StudentTemplate.slx`.
2. Click the green **Run** button in the Simulink toolstrip.
3. Simulink will automatically:
   * Start a Pygame maze simulation in a background desktop process.
   * Establish a network connection with the simulator.
   * Step the virtual mouse through the maze.
4. Click **Stop** in Simulink to automatically close the visualization window.

---

## 💾 Step 3: Physical Connections & Safety

To protect your hardware, strictly follow the safety rules:
1. **Single USB Cable Only:** Connect **one** USB-C cable to the port labeled **ST-LINK** (the programmer debugger board). Do not plug multiple USB cables into the board or chassis at the same time.
2. **Do Not Touch the Wheels:** The motors have high-ratio gearboxes. Spinning the wheels by hand will strip the gears and permanently destroy them.
3. **Turn the Battery On (If connected to chassis):** Ensure the battery is charged and the physical switch on the mouse chassis is turned **ON** so the sensors and motors are powered.

---

## 🚀 Step 4: Compile and Deploy to the STM32

Once your model works in simulation, compile it directly into STM32 assembly/binary:

1. Open `matlab/simulink/UCT_KDeploy.slx`.
2. This is the hardware deployment model which references `StudentTemplate.slx`.
3. Press **`Cmd + B`** (macOS) or **`Ctrl + B`** (Windows) to trigger compilation.
4. MATLAB's Embedded Coder will compile the C-Kernel and inject your controller logic.
5. Once compilation is complete, flash the binary directly to the board. If you do not have an ST-LINK programmer, you can flash it over the **USB OTG** port using DFU:
   * Bridge the `BOOT0` header pin to `3V3` (using a jumper or screwdriver tip) while pressing and releasing the **RESET** button to enter DFU mode.
   * Run the deployment script from your terminal:
     ```bash
     python3 tools/deploy.py --engine simulink --flash
     ```
6. Disconnect the USB cable, place the mouse on the ground, and press the black **RESET** button to run your compiled Simulink model!

---

## 🔍 Troubleshooting

* **Build Errors in Simulink:**
  Ensure you ran the `startup.m` script. If paths are missing, Embedded Coder will fail to find custom C-Kernel headers.
* **USB Flash Failure:**
  Ensure the ST-LINK programmer board is plugged in, and its LED turns green. If it hangs, unplug and replug the programmer USB cable.

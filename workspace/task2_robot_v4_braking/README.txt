Snapshot of workspace/task2_robot taken 2026-10-23 (v4) - completed the maze.
v3 (square-up) plus: GYRO_SCALE 0.992 turn calibration, TURN_LAG 0.14 /
TURN_W_MIN 0.8, the wall-slope parallel check, active per-wheel braking at
the end of drives, re-align before a drive if > 1.2 deg off.

To go back to it:
  1. copy the .py files from this folder over workspace/task2_robot/
  2. python workspace/task2_robot/build_robot.py --port COM10

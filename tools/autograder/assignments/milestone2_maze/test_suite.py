import json
import math

# Milestone 2 / Submission 4 parameters: 4x6 / 8x8 maze with 2x2 target room
MAP = "random"
TIME_LIMIT = 120.0
IMBALANCE = 0.08
SLIP = 0.02
SEED = 42

# Maze Geometry Defaults: 4 rows x 6 cols (0.8m x 1.2m)
CELL_DIM = 0.20
MAZE_ROWS = 4
MAZE_COLS = 6

# Multi-Tier Excellence Evaluation Matrix
# format: (name, weight, imbalance, slip, is_hidden, rows, cols, seed, max_time)
TEST_RUNS = [
    ("Test 1: Baseline Maze Navigation (4x6, Seed 42)", 0.20, 0.04, 0.02, False, 4, 6, 42, 120.0),
    ("Test 2: Asymmetric Corridors & Motor Bias (+6%, Seed 43)", 0.20, 0.06, 0.02, False, 4, 6, 43, 120.0),
    ("Test 3: Complex Non-Convex Alternate Path (-6%, Seed 44)", 0.20, -0.06, 0.02, False, 4, 6, 44, 120.0),
    ("Test 4: Dynamic Traction Slip & Corridor Merging (4%, Seed 45)", 0.20, 0.04, 0.04, False, 4, 6, 45, 120.0),
    ("Test 5: Championship Scalability Arena (8x8, Seed 48)", 0.20, 0.05, 0.03, False, 8, 8, 48, 240.0),
]

def evaluate_run(trajectory_file):
    try:
        with open(trajectory_file, "r") as f:
            data = json.load(f)
    except Exception as e:
        return 0.0, f"Error reading trajectory file: {e}"

    start_x = data.get("start_x", 0.10)
    start_y = data.get("start_y", 0.10)
    final_x = data.get("final_x", start_x)
    final_y = data.get("final_y", start_y)
    sim_time = data.get("time", 0.0)
    crashed = data.get("crashed", False)
    trajectory = data.get("trajectory", []) # [(x, y, theta, t), ...]
    
    grid_rows = data.get("grid_rows", MAZE_ROWS)
    grid_cols = data.get("grid_cols", MAZE_COLS)
    maze_w = grid_cols * CELL_DIM
    maze_h = grid_rows * CELL_DIM

    # Resolve target room center dynamically from simulator log or default geometry
    if "target_room" in data and data["target_room"]:
        cx, cy = data["target_room"]
        target_center_x = cx * CELL_DIM
        target_center_y = cy * CELL_DIM
    else:
        target_center_x = (grid_cols - 2) * CELL_DIM
        target_center_y = (grid_rows // 2) * CELL_DIM
    target_radius = 0.28 # Within 2x2 room bounding circle

    # Check Stage 1: Target Discovery (Did trajectory enter target zone?)
    min_dist_to_target = 999.0
    entered_target = False
    target_entry_idx = -1

    for idx, pt in enumerate(trajectory):
        d = math.hypot(pt[0] - target_center_x, pt[1] - target_center_y)
        if d < min_dist_to_target:
            min_dist_to_target = d
        if d <= target_radius and not entered_target:
            entered_target = True
            target_entry_idx = idx

    # Check Stage 2: 360-degree Recognition Pirouette inside target zone
    # Must be an ON-THE-SPOT spin (high yaw accumulation with minimal linear translation)
    pirouette_detected = False
    pirouette_end_idx = -1
    if entered_target:
        for start_i in range(target_entry_idx, len(trajectory) - 5):
            if math.hypot(trajectory[start_i][0] - target_center_x, trajectory[start_i][1] - target_center_y) > target_radius:
                continue
            
            yaw_accum = 0.0
            spin_start_pt = trajectory[start_i]
            prev_theta = spin_start_pt[2]
            max_linear_disp = 0.0
            
            for end_i in range(start_i + 1, min(len(trajectory), start_i + 350)):
                pt = trajectory[end_i]
                d_theta = pt[2] - prev_theta
                while d_theta > math.pi: d_theta -= 2 * math.pi
                while d_theta < -math.pi: d_theta += 2 * math.pi
                yaw_accum += abs(d_theta)
                prev_theta = pt[2]
                
                disp = math.hypot(pt[0] - spin_start_pt[0], pt[1] - spin_start_pt[1])
                if disp > max_linear_disp:
                    max_linear_disp = disp
                
                # Check criteria: >= 315° to 360° total rotation while translating < 12 cm
                if yaw_accum >= 5.5 and max_linear_disp <= 0.12:
                    pirouette_detected = True
                    pirouette_end_idx = end_i
                    break
            if pirouette_detected:
                break

    # Check Stage 3: Return to Start (0,0) after visiting target
    returned_to_start = False
    return_entry_idx = -1
    search_after_idx = pirouette_end_idx if pirouette_end_idx >= 0 else target_entry_idx
    if entered_target and search_after_idx >= 0:
        for idx in range(search_after_idx, len(trajectory)):
            pt = trajectory[idx]
            d_start = math.hypot(pt[0] - start_x, pt[1] - start_y)
            if d_start <= 0.15:
                returned_to_start = True
                return_entry_idx = idx
                break

    # Check Stage 4: High-Speed Sprint back to target
    sprint_completed = False
    sprint_start_idx = -1
    sprint_end_idx = -1
    if returned_to_start and return_entry_idx >= 0:
        # Find when the mouse departs start cell after return
        for idx in range(return_entry_idx, len(trajectory)):
            pt = trajectory[idx]
            d_start = math.hypot(pt[0] - start_x, pt[1] - start_y)
            if d_start > 0.15 and sprint_start_idx < 0:
                sprint_start_idx = idx
            if sprint_start_idx >= 0:
                d_target = math.hypot(pt[0] - target_center_x, pt[1] - target_center_y)
                if d_target <= target_radius:
                    sprint_completed = True
                    sprint_end_idx = idx
                    break

        if not sprint_completed:
            final_dist_to_target = math.hypot(final_x - target_center_x, final_y - target_center_y)
            if final_dist_to_target <= target_radius:
                sprint_completed = True
                sprint_end_idx = len(trajectory) - 1

    feedback = []
    feedback.append("=== Submission 4: Final Maze Solver Evaluation ===")
    feedback.append(f"Maze Arena Geometry : {grid_rows} rows x {grid_cols} cols ({maze_h:.2f}m x {maze_w:.2f}m)")
    feedback.append(f"Start Position      : ({start_x:.3f}, {start_y:.3f})")
    feedback.append(f"Final Position      : ({final_x:.3f}, {final_y:.3f})")
    feedback.append(f"Min Dist to Target  : {min_dist_to_target:.3f} m")
    feedback.append(f"Simulation Time     : {sim_time:.2f} s")
    feedback.append(f"Collision / Crash   : {'YES' if crashed else 'NO'}")

    # Point allocation (100 pts total)
    # Stage 1: Target Discovery (30 pts)
    if entered_target:
        score_target = 30.0
        feedback.append("  [Phase 1] SUCCESS: 2x2 Target Room discovered and entered (30.0 / 30.0 pts)")
    else:
        progress = max(0.0, 1.0 - min_dist_to_target / max(1.0, math.hypot(maze_w, maze_h)))
        score_target = 30.0 * progress
        feedback.append(f"  [Phase 1] PARTIAL: Target room not reached ({score_target:.1f} / 30.0 pts)")

    # Stage 2: Pirouette (20 pts)
    score_pirouette = 20.0 if pirouette_detected else 0.0
    feedback.append(f"  [Phase 2] {'SUCCESS: 360° Target Recognition Pirouette verified' if pirouette_detected else 'FAILED: No 360° pirouette detected inside target room'} ({score_pirouette:.1f} / 20.0 pts)")

    # Stage 3: Return to Start (20 pts)
    score_return = 20.0 if returned_to_start else 0.0
    feedback.append(f"  [Phase 3] {'SUCCESS: Autonomous return to start (0,0) verified' if returned_to_start else 'FAILED: Did not return to start cell (0,0)'} ({score_return:.1f} / 20.0 pts)")

    # Stage 4: Sprint to Target (20 pts)
    score_sprint = 20.0 if sprint_completed else 0.0
    feedback.append(f"  [Phase 4] {'SUCCESS: High-speed sprint completed to target' if sprint_completed else 'FAILED: High-speed sprint to target not completed'} ({score_sprint:.1f} / 20.0 pts)")

    # Speed & Dynamic Optimization Bonus (10 pts total)
    # Divided into:
    # 1. Absolute Sprint Velocity (5 pts): average speed during Phase 4 sprint
    # 2. Trajectory Speedup Factor (5 pts): sprint speed relative to exploration speed
    score_vel_bonus = 0.0
    score_speedup_bonus = 0.0
    v_sprint = 0.0
    v_explore = 0.0
    speedup_ratio = 1.0

    if sprint_completed and not crashed and len(trajectory) >= 2:
        # Calculate Exploration average velocity
        exp_end = target_entry_idx if target_entry_idx > 0 else len(trajectory) // 3
        d_exp = sum(math.hypot(trajectory[k][0] - trajectory[k-1][0], trajectory[k][1] - trajectory[k-1][1]) for k in range(1, exp_end))
        t_exp_0 = trajectory[0][3] if len(trajectory[0]) > 3 else 0.0
        t_exp_end = trajectory[exp_end][3] if len(trajectory[exp_end]) > 3 else exp_end * 0.05
        t_exp = max(0.5, t_exp_end - t_exp_0)
        v_explore = d_exp / t_exp

        # Calculate Sprint average velocity
        sp_start = sprint_start_idx if sprint_start_idx > 0 else (return_entry_idx if return_entry_idx > 0 else 0)
        sp_end = sprint_end_idx if sprint_end_idx > sp_start else len(trajectory) - 1
        d_sprint = sum(math.hypot(trajectory[k][0] - trajectory[k-1][0], trajectory[k][1] - trajectory[k-1][1]) for k in range(sp_start + 1, sp_end + 1))
        t_sp_0 = trajectory[sp_start][3] if len(trajectory[sp_start]) > 3 else sp_start * 0.05
        t_sp_end = trajectory[sp_end][3] if len(trajectory[sp_end]) > 3 else sp_end * 0.05
        t_sprint = max(0.5, t_sp_end - t_sp_0)
        v_sprint = d_sprint / t_sprint

        speedup_ratio = v_sprint / max(0.06, v_explore)

        # Bonus 1: Absolute Sprint Velocity (5.0 pts)
        # Full marks at >= 0.30 m/s; 0 marks at <= 0.15 m/s
        if v_sprint >= 0.30:
            score_vel_bonus = 5.0
        elif v_sprint > 0.15:
            score_vel_bonus = 5.0 * (v_sprint - 0.15) / (0.30 - 0.15)

        # Bonus 2: Sprint Speedup Factor (5.0 pts)
        # Full marks at >= 1.35x speedup over exploration; 0 marks at <= 1.05x (identical motion)
        if speedup_ratio >= 1.35:
            score_speedup_bonus = 5.0
        elif speedup_ratio > 1.05:
            score_speedup_bonus = 5.0 * (speedup_ratio - 1.05) / (1.35 - 1.05)

    speed_bonus = min(10.0, score_vel_bonus + score_speedup_bonus)
    feedback.append(f"  [Bonus A] Absolute Sprint Velocity: {score_vel_bonus:.1f} / 5.0 pts ({v_sprint:.3f} m/s, Target >= 0.30 m/s)")
    feedback.append(f"  [Bonus B] Trajectory Speedup Factor: {score_speedup_bonus:.1f} / 5.0 pts ({speedup_ratio:.2f}x vs exploration, Target >= 1.35x)")

    subtotal = score_target + score_pirouette + score_return + score_sprint + speed_bonus
    active_limit = 240.0 if grid_rows >= 8 else TIME_LIMIT
    timeout_penalty = 10.0 if sim_time >= active_limit else 0.0
    if timeout_penalty > 0:
        feedback.append(f"  [Penalty] Timeout Penalty (> {active_limit:.0f}s): -{timeout_penalty:.1f} pts")

    final_grade = max(0.0, min(100.0, subtotal - timeout_penalty))
    final_grade_rounded = round(final_grade)

    feedback.append("\n=== Total Score Arithmetic ===")
    feedback.append(f"  Phase 1 (Target Discovery)     : {score_target:5.1f} / 30.0 pts")
    feedback.append(f"  Phase 2 (360° Pirouette)       : {score_pirouette:5.1f} / 20.0 pts")
    feedback.append(f"  Phase 3 (Return to Start)      : {score_return:5.1f} / 20.0 pts")
    feedback.append(f"  Phase 4 (High-Speed Sprint)    : {score_sprint:5.1f} / 20.0 pts")
    feedback.append(f"  Sprint Velocity & Speedup Bonus: {speed_bonus:5.1f} / 10.0 pts")
    feedback.append(f"  -------------------------------------------")
    feedback.append(f"  Final Calculated Grade         : {final_grade_rounded}%")

    return float(final_grade_rounded), "\n".join(feedback)

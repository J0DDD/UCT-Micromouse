import json
import math

def unwrap_angles(thetas):
    if not thetas:
        return []
    res = [thetas[0]]
    for t in thetas[1:]:
        diff = t - res[-1]
        diff = (diff + math.pi) % (2.0 * math.pi) - math.pi
        res.append(res[-1] + diff)
    return res

# Milestone 1 parameters
MAP = "empty"
TIME_LIMIT = 50.0
SEED = 42

# Calibrated 6-Test Evaluation Suite (25% Public Baseline, 5x15% Hidden Stress Tests)
# Public Test 1 carries 25% weight (15.0/60 pts) and uses standard rubric.
# Hidden Tests 2-6 carry 15% weight each (9.0/60 pts each) with continuous linear deductions.
# All physical parameters realistically bounded: Imbalance <= 12%, Slip <= 12%
# format: (name, weight, imbalance, slip, is_hidden)
TEST_RUNS = [
    ("Test 1: Public Baseline Run", 0.25, 0.05, 0.03, False),
    ("Test 2: Hidden Positive Imbalance (+12%)", 0.15, 0.12, 0.06, True),
    ("Test 3: Hidden Negative Imbalance (-12%)", 0.15, -0.12, 0.06, True),
    ("Test 4: Hidden Traction Slip & Spin (12% Slip)", 0.15, 0.06, 0.12, True),
    ("Test 5: Hidden Turn Settling & Dynamic Skid", 0.15, -0.12, 0.10, True),
    ("Test 6: Hidden Compound Perturbation", 0.15, 0.12, 0.12, True)
]

def evaluate_run(trajectory_file, is_hidden=False):
    try:
        with open(trajectory_file, "r") as f:
            data = json.load(f)
    except Exception as e:
        return 0.0, f"Error reading trajectory file: {e}"

    start_x = data.get("start_x", 0.0)
    start_y = data.get("start_y", 0.0)
    final_x = data.get("final_x", 0.0)
    final_y = data.get("final_y", 0.0)
    sim_time = data.get("time", 0.0)
    crashed = data.get("crashed", False)
    trajectory = data.get("trajectory", [])

    feedback = []
    feedback.append("=== Milestone 1 Trajectory Profile Evaluation ===")
    feedback.append(f"Simulation Time  : {sim_time:.2f} s")
    feedback.append(f"Collision State  : {'CRASHED' if crashed else 'CLEAN RUN'}")
    
    if len(trajectory) < 10:
        return 0.0, "Trajectory data incomplete or too short to analyze (mouse did not move)."

    # Compute overall path distance and bounding box coverage
    total_dist = 0.0
    for k in range(1, len(trajectory)):
        total_dist += math.hypot(trajectory[k][0] - trajectory[k-1][0], trajectory[k][1] - trajectory[k-1][1])
        
    xs = [pt[0] for pt in trajectory]
    ys = [pt[1] for pt in trajectory]
    dx = max(xs) - min(xs)
    dy = max(ys) - min(ys)
    bbox_diag = math.hypot(dx, dy)
    
    feedback.append(f"Total Traversed  : {total_dist:.2f} m (Target: ~4.00 m)")
    feedback.append(f"Bounding Box     : {dx:.2f} m × {dy:.2f} m (Diagonal: {bbox_diag:.2f} m)")

    # Kinematic Classification: Segment into Translating Legs (1), Turning Corners (2), and Settling (0)
    raw_thetas = [pt[2] for pt in trajectory]
    unwrapped = unwrap_angles(raw_thetas)
    
    classes = []
    for k in range(len(trajectory)):
        k_prev = max(0, k - 2)
        k_next = min(len(trajectory) - 1, k + 2)
        # Approximate local time delta: ~0.1s per step
        dt = max(0.02, (k_next - k_prev) * 0.05)
        d_dist = math.hypot(trajectory[k_next][0] - trajectory[k_prev][0], trajectory[k_next][1] - trajectory[k_prev][1])
        d_th = abs(unwrapped[k_next] - unwrapped[k_prev])
        
        v = d_dist / dt
        omega = d_th / dt
        
        if omega > math.radians(18.0):
            classes.append(2)  # Active turn
        elif v > 0.03:
            classes.append(1)  # Translating forward
        else:
            classes.append(0)  # Settle / stop
            
    # Group consecutive classes into clusters
    clusters = []
    if classes:
        cur_c = classes[0]
        cur_pts = [trajectory[0]]
        for k in range(1, len(trajectory)):
            if classes[k] == cur_c:
                cur_pts.append(trajectory[k])
            else:
                clusters.append((cur_c, cur_pts))
                cur_c = classes[k]
                cur_pts = [trajectory[k]]
        clusters.append((cur_c, cur_pts))
        
    # Extract significant turn segments (filtering short transients < 3 steps)
    turn_clusters = [pts for c_type, pts in clusters if c_type == 2 and len(pts) >= 3]

    # Segment translating legs between turns:
    # Legs are the physical path intervals between turns, preventing startup hesitations from splitting legs.
    leg_clusters = []
    if not turn_clusters:
        if len(trajectory) >= 3:
            leg_clusters.append(trajectory)
    else:
        # Leg 1: Start of trajectory up to start of Turn 1
        t0_start_idx = trajectory.index(turn_clusters[0][0])
        if t0_start_idx >= 1:
            leg_clusters.append(trajectory[:t0_start_idx + 1])
        else:
            leg_clusters.append(trajectory[:max(1, t0_start_idx)])
            
        # Intermediate legs between turns
        for t_idx in range(len(turn_clusters) - 1):
            if len(leg_clusters) >= 4:
                break
            t_prev_end_idx = trajectory.index(turn_clusters[t_idx][-1])
            t_next_start_idx = trajectory.index(turn_clusters[t_idx + 1][0])
            if t_next_start_idx > t_prev_end_idx:
                leg_clusters.append(trajectory[t_prev_end_idx:t_next_start_idx + 1])
                
        # Final leg: After the last detected turn (if < 4 turns were detected)
        if len(turn_clusters) < 4 and len(leg_clusters) < 4:
            t_last_end_idx = trajectory.index(turn_clusters[-1][-1])
            if len(trajectory) > t_last_end_idx:
                leg_clusters.append(trajectory[t_last_end_idx:])
        elif len(turn_clusters) >= 4 and len(leg_clusters) < 4:
            t2_end_idx = trajectory.index(turn_clusters[2][-1])
            t3_start_idx = trajectory.index(turn_clusters[3][0])
            if t3_start_idx > t2_end_idx:
                leg_clusters.append(trajectory[t2_end_idx:t3_start_idx + 1])

    # Evaluate the 4 Straight Line Segments (40 points total - 10.0 points per leg: 6.0 dist + 4.0 straightness)
    leg_scores = []
    valid_legs_count = 0
    feedback.append("\n--- Leg Trajectory Analysis (Straightness & Length) ---")
    for i in range(4):
        if i >= len(leg_clusters):
            feedback.append(f"  Leg {i+1}: Not executed. Scored 0.00/10.00")
            leg_scores.append(0.0)
            continue
            
        pts = leg_clusters[i]
        leg_len = math.hypot(pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1])
        len_error = abs(leg_len - 1.0)
        
        if leg_len < 0.10:
            len_score = 0.0
        else:
            if not is_hidden:
                if len_error <= 0.05:
                    len_score = 6.00
                else:
                    len_score = max(0.0, 6.00 - (len_error - 0.05) / 0.20 * 6.00)
            else:
                len_score = max(0.0, 6.00 * (1.0 - len_error / 0.18))
            
        x0, y0 = pts[0][0], pts[0][1]
        x1, y1 = pts[-1][0], pts[-1][1]
        line_len = math.hypot(x1 - x0, y1 - y0)
        
        max_dev = 0.0
        if line_len > 1e-3:
            for pt in pts:
                px, py = pt[0], pt[1]
                dev = abs((y1 - y0) * px - (x1 - x0) * py + x1 * y0 - y1 * x0) / line_len
                max_dev = max(max_dev, dev)
                
        if leg_len < 0.35:
            straight_score = 0.0
        else:
            length_fraction = min(1.0, leg_len / 0.80)
            if not is_hidden:
                if max_dev <= 0.03:
                    raw_straight = 4.00
                else:
                    raw_straight = max(0.0, 4.00 - (max_dev - 0.03) / 0.12 * 4.00)
            else:
                raw_straight = max(0.0, 4.00 * (1.0 - max_dev / 0.10))
            straight_score = raw_straight * length_fraction
            
        if leg_len >= 0.40:
            valid_legs_count += 1

        leg_score = len_score + straight_score
        leg_scores.append(leg_score)
        feedback.append(f"  Leg {i+1} ({['East', 'North', 'West', 'South'][i]}): Length={leg_len:.2f}m (err={len_error*100:.1f}cm), Max Dev={max_dev*100:.1f}cm -> Score {leg_score:.2f}/10.00")

    # Evaluate the 4 Turns / Right-Angleness (30 points total - 7.5 points per corner)
    turn_scores = []
    valid_turns_count = 0
    feedback.append("\n--- Corner Analysis (Right-Angleness) ---")
    for i in range(4):
        if i >= len(turn_clusters):
            feedback.append(f"  Corner {i+1}: Turn not detected. Scored 0.00/7.50")
            turn_scores.append(0.0)
            continue
            
        pts = turn_clusters[i]
        idx0 = trajectory.index(pts[0])
        idx1 = trajectory.index(pts[-1])
        turn_angle = abs(unwrapped[idx1] - unwrapped[idx0])
        turn_deg = math.degrees(turn_angle)
        turn_error = abs(turn_deg - 90.0)
        
        if not is_hidden:
            if turn_error <= 4.0:
                t_score = 7.50
            else:
                t_score = max(0.0, 7.50 - (turn_error - 4.0) / 14.0 * 7.50)
        else:
            t_score = max(0.0, 7.50 * (1.0 - turn_error / 12.0))
            
        if t_score > 1.0:
            valid_turns_count += 1
            
        turn_scores.append(t_score)
        feedback.append(f"  Corner {i+1} ({['E->N', 'N->W', 'W->S', 'S->E'][i]}): Turn Angle={turn_deg:.1f}° (err={turn_error:.1f}°) -> Score {t_score:.2f}/7.50")

    # Return & Parking accuracy (25 points)
    # GATED: Closure is only evaluated if the mouse completed at least 3 valid legs and total_dist >= 2.5m
    feedback.append("\n--- Circuit Closure & Parking Accuracy ---")
    if valid_legs_count < 3 or total_dist < 2.5:
        parking_score = 0.0
        feedback.append(f"  Return Offset : Gated (0.00/25.00). Must traverse at least 3 legs and 2.5m of perimeter (traversed: {total_dist:.2f}m, valid legs: {valid_legs_count}).")
    else:
        # Search candidate points from 2.5m perimeter traversal onwards
        cum_dist = 0.0
        k_search_start = 0
        for k in range(1, len(trajectory)):
            cum_dist += math.hypot(trajectory[k][0] - trajectory[k-1][0], trajectory[k][1] - trajectory[k-1][1])
            if cum_dist >= 2.5 and k_search_start == 0:
                k_search_start = k
                
        candidate_points = trajectory[k_search_start:] if k_search_start > 0 else trajectory[-5:]
        d_e = min(math.hypot(p[0] - start_x, p[1] - start_y) for p in candidate_points)

        if not is_hidden:
            if d_e <= 0.05:
                parking_score = 25.0
            else:
                parking_score = max(0.0, 25.0 - (d_e - 0.05) / 0.25 * 25.0)
        else:
            parking_score = max(0.0, 25.0 * (1.0 - d_e / 0.18))
            
        feedback.append(f"  Return Offset (at Square Completion): {d_e*100:.1f} cm -> Parking Score {parking_score:.2f}/25.00")
        if math.hypot(final_x - start_x, final_y - start_y) > d_e + 0.02:
            feedback.append("  [Info] Extra forward rollout beyond origin incurs 0 penalty.")

    # Efficiency (5 points total speed score)
    feedback.append("\n--- Efficiency ---")
    if sim_time <= 30.0:
        raw_speed = 5.00
    else:
        raw_speed = max(0.0, 5.00 - (sim_time - 30.0) / 20.0 * 5.00)
    speed_score = raw_speed * (valid_legs_count / 4.0)
        
    feedback.append(f"  Speed Score (time={sim_time:.1f}s, completion={valid_legs_count}/4): {speed_score:.2f}/5.00")

    # Final Grade Calculation
    base_legs = sum(leg_scores)
    base_turns = sum(turn_scores)
    total_grade = base_legs + base_turns + parking_score + speed_score
    
    final_grade_rounded = round(total_grade)

    feedback.append("\n=== Score Arithmetic Breakdown ===")
    feedback.append(f"  Leg Segments (Length + Straightness): {base_legs:5.2f} / 40.00 pts")
    feedback.append(f"  Corner Turn Angles (90° accuracy)   : {base_turns:5.2f} / 30.00 pts")
    feedback.append(f"  Circuit Closure (Return to start)   : {parking_score:5.2f} / 25.00 pts")
    feedback.append(f"  Run Speed Efficiency (Completed)    : {speed_score:5.2f} /  5.00 pts")
    feedback.append(f"  -------------------------------------------")
    feedback.append(f"  Calculated Grade                    : {total_grade:5.2f} / 100.00 pts")
    feedback.append(f"  GRADE: {final_grade_rounded}%")

    return float(final_grade_rounded), "\n".join(feedback)

import json
import math
import numpy as np

# Milestone 1 parameters
MAP = "empty"
TIME_LIMIT = 45.0
SEED = 42

# Define multiple evaluation test runs
# format: (name, weight, imbalance, slip, is_hidden)
TEST_RUNS = [
    ("Test 1: Public Baseline Run", 0.40, 0.05, 0.04, False),
    ("Test 2: Hidden Asymmetry Stress-Test", 0.30, 0.12, 0.02, True),
    ("Test 3: Hidden Starting/Turning Slip Run", 0.30, 0.04, 0.10, True)
]

def evaluate_run(trajectory_file):
    try:
        with open(trajectory_file, "r") as f:
            # Trajectory is stored as a list of dicts or standard summary format
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
        return 0.0, "Trajectory data incomplete or too short to analyze."

    # Segment the trajectory chronologically into 4 straight legs and 4 turns
    # The mouse transitions orientation CCW: Leg 0 (0 rad) -> Turn 0 -> Leg 1 (pi/2) -> Turn 1 -> Leg 2 (pi) -> Turn 2 -> Leg 3 (-pi/2) -> Turn 3 -> Stop
    target_headings = [0.0, math.pi / 2.0, math.pi, -math.pi / 2.0]
    
    legs_points = {0: [], 1: [], 2: [], 3: []}
    turns_points = {0: [], 1: [], 2: [], 3: []}
    
    # State tracking: 0=Leg1, 1=Turn1, 2=Leg2, 3=Turn2, 4=Leg3, 5=Turn3, 6=Leg4, 7=Turn4, 8=Finished
    current_state = 0
    turn4_end_heading = None
    
    for pt in trajectory:
        tx, ty, ttheta = pt[0], pt[1], pt[2]
        # Wrap theta to [-pi, pi]
        ttheta = (ttheta + math.pi) % (2.0 * math.pi) - math.pi
        
        if current_state == 0:
            # Leg 1: Target heading 0.0 (East)
            err = abs((ttheta - 0.0 + math.pi) % (2.0 * math.pi) - math.pi)
            if err < math.radians(25.0):
                legs_points[0].append((tx, ty, ttheta))
            else:
                if len(legs_points[0]) >= 3:
                    current_state = 1
                    turns_points[0].append((tx, ty, ttheta))
                else:
                    legs_points[0].append((tx, ty, ttheta))
        elif current_state == 1:
            # Turn 1: Transitioning East -> North (pi/2)
            err_next = abs((ttheta - (math.pi / 2.0) + math.pi) % (2.0 * math.pi) - math.pi)
            if err_next < math.radians(25.0):
                current_state = 2
                legs_points[1].append((tx, ty, ttheta))
            else:
                turns_points[0].append((tx, ty, ttheta))
        elif current_state == 2:
            # Leg 2: Target heading pi/2 (North)
            err = abs((ttheta - (math.pi / 2.0) + math.pi) % (2.0 * math.pi) - math.pi)
            if err < math.radians(25.0):
                legs_points[1].append((tx, ty, ttheta))
            else:
                if len(legs_points[1]) >= 3:
                    current_state = 3
                    turns_points[1].append((tx, ty, ttheta))
                else:
                    legs_points[1].append((tx, ty, ttheta))
        elif current_state == 3:
            # Turn 2: Transitioning North -> West (pi)
            err_next = abs((ttheta - math.pi + math.pi) % (2.0 * math.pi) - math.pi)
            if err_next < math.radians(25.0):
                current_state = 4
                legs_points[2].append((tx, ty, ttheta))
            else:
                turns_points[1].append((tx, ty, ttheta))
        elif current_state == 4:
            # Leg 3: Target heading pi (West)
            err = abs((ttheta - math.pi + math.pi) % (2.0 * math.pi) - math.pi)
            if err < math.radians(25.0):
                legs_points[2].append((tx, ty, ttheta))
            else:
                if len(legs_points[2]) >= 3:
                    current_state = 5
                    turns_points[2].append((tx, ty, ttheta))
                else:
                    legs_points[2].append((tx, ty, ttheta))
        elif current_state == 5:
            # Turn 3: Transitioning West -> South (-pi/2)
            err_next = abs((ttheta - (-math.pi / 2.0) + math.pi) % (2.0 * math.pi) - math.pi)
            if err_next < math.radians(25.0):
                current_state = 6
                legs_points[3].append((tx, ty, ttheta))
            else:
                turns_points[2].append((tx, ty, ttheta))
        elif current_state == 6:
            # Leg 4: Target heading -pi/2 (South)
            err = abs((ttheta - (-math.pi / 2.0) + math.pi) % (2.0 * math.pi) - math.pi)
            if err < math.radians(25.0):
                legs_points[3].append((tx, ty, ttheta))
            else:
                if len(legs_points[3]) >= 3:
                    current_state = 7
                    turns_points[3].append((tx, ty, ttheta))
                else:
                    legs_points[3].append((tx, ty, ttheta))
        elif current_state == 7:
            # Turn 4: Transitioning South -> East (0.0 / Finish)
            err_next = abs((ttheta - 0.0 + math.pi) % (2.0 * math.pi) - math.pi)
            if err_next < math.radians(25.0):
                current_state = 8
                turn4_end_heading = ttheta
            else:
                turns_points[3].append((tx, ty, ttheta))
        elif current_state == 8:
            # Finished square, parked at origin (do not append to Leg 1!)
            turn4_end_heading = ttheta

    # If the run ended during Turn 4 or after stopping without moving forward along a 5th leg,
    # capture the final heading from the last trajectory point.
    if turn4_end_heading is None and len(trajectory) > 0:
        if current_state >= 7 or len(turns_points[3]) > 0:
            final_theta = trajectory[-1][2]
            turn4_end_heading = (final_theta + math.pi) % (2.0 * math.pi) - math.pi

    # Evaluate the 4 Straight Line Segments (30 points total - 7.5 points per leg)
    leg_scores = []
    feedback.append("\n--- Leg Trajectory Analysis (Straightness & Length) ---")
    for i in range(4):
        pts = legs_points[i]
        if len(pts) < 3:
            feedback.append(f"  Leg {i+1}: Insufficient trajectory points. Scored 0.0/7.5")
            leg_scores.append(0.0)
            continue
            
        # Calculate length (Euclidean distance between start and end of leg)
        leg_len = math.hypot(pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1])
        len_error = abs(leg_len - 1.0)
        
        # Score length (out of 3.75 points): full score if error <= 5cm, scales to 0 at 25cm
        if len_error <= 0.05:
            len_score = 3.75
        else:
            len_score = max(0.0, 3.75 - (len_error - 0.05) / 0.20 * 3.75)
            
        # Calculate straightness (maximum lateral deviation from ideal straight line vector)
        p0 = np.array([pts[0][0], pts[0][1]])
        p1 = np.array([pts[-1][0], pts[-1][1]])
        line_vec = p1 - p0
        line_len = np.linalg.norm(line_vec)
        
        max_dev = 0.0
        if line_len > 1e-3:
            for pt in pts:
                p = np.array([pt[0], pt[1]])
                # Perpendicular distance from point p to line segment p0-p1
                dev = np.abs(np.cross(line_vec, p0 - p)) / line_len
                max_dev = max(max_dev, dev)
                
        # Score straightness (out of 3.75 points): full score if max deviation <= 2cm, scales to 0 at 15cm
        if max_dev <= 0.02:
            straight_score = 3.75
        else:
            straight_score = max(0.0, 3.75 - (max_dev - 0.02) / 0.13 * 3.75)
            
        leg_score = len_score + straight_score
        leg_scores.append(leg_score)
        feedback.append(f"  Leg {i+1} ({['East', 'North', 'West', 'South'][i]}): Length={leg_len:.2f}m (err={len_error*100:.1f}cm), Max Dev={max_dev*100:.1f}cm -> Score {leg_score:.2f}/7.50")

    # Calculate representative straight heading for each of the 4 legs (circular mean)
    def circular_mean(thetas):
        sin_sum = sum(math.sin(th) for th in thetas)
        cos_sum = sum(math.cos(th) for th in thetas)
        return math.atan2(sin_sum, cos_sum)

    leg_headings = {}
    for i in range(4):
        if len(legs_points[i]) > 0:
            leg_headings[i] = circular_mean([pt[2] for pt in legs_points[i]])
        else:
            leg_headings[i] = target_headings[i]

    # Evaluate the 4 Turns / Right-Angleness (30 points total - 7.5 points per corner)
    turn_scores = []
    feedback.append("\n--- Corner Analysis (Right-Angleness) ---")
    for i in range(4):
        if len(legs_points[i]) < 3:
            feedback.append(f"  Corner {i+1}: Incomplete turn trajectory (missing Leg {i+1}). Scored 0.0/7.5")
            turn_scores.append(0.0)
            continue
            
        h_start = leg_headings[i]
        
        if i < 3:
            if len(legs_points[i + 1]) < 3:
                feedback.append(f"  Corner {i+1}: Incomplete turn trajectory (missing Leg {i+2}). Scored 0.0/7.5")
                turn_scores.append(0.0)
                continue
            h_end = leg_headings[i + 1]
        else:
            # Corner 4: Turn from Leg 4 to the final completed heading at the origin
            if turn4_end_heading is not None:
                h_end = turn4_end_heading
            elif turns_points[3]:
                h_end = turns_points[3][-1][2]
            elif len(trajectory) > 0:
                final_theta = trajectory[-1][2]
                h_end = (final_theta + math.pi) % (2.0 * math.pi) - math.pi
            else:
                feedback.append(f"  Corner {i+1}: Incomplete turn trajectory. Scored 0.0/7.5")
                turn_scores.append(0.0)
                continue
        
        turn_angle = (h_end - h_start + math.pi) % (2.0 * math.pi) - math.pi
        # Wrap CCW angle to positive degrees
        turn_deg = abs(math.degrees(turn_angle))
        
        # Error from ideal 90 degree turn
        turn_error = abs(turn_deg - 90.0)
        
        # Score (out of 7.5 points): full score if error <= 3 degrees, scales to 0 at 15 degrees
        if turn_error <= 3.0:
            t_score = 7.5
        else:
            t_score = max(0.0, 7.5 - (turn_error - 3.0) / 12.0 * 7.5)
            
        turn_scores.append(t_score)
        feedback.append(f"  Corner {i+1} ({['E->N', 'N->W', 'W->S', 'S->E'][i]}): Turn Angle={turn_deg:.1f}° (err={turn_error:.1f}°) -> Score {t_score:.2f}/7.50")

    # Return & Parking accuracy (20 points)
    feedback.append("\n--- Return & Parking Accuracy ---")
    d_e = math.hypot(final_x - start_x, final_y - start_y)
    # Full points if final distance to start is <= 3cm, scales to 0 at 25cm
    if d_e <= 0.03:
        parking_score = 20.0
    else:
        parking_score = max(0.0, 20.0 - (d_e - 0.03) / 0.22 * 20.0)
    feedback.append(f"  Final Position Offset: {d_e*100:.1f} cm -> Parking Score {parking_score:.2f}/20.0")

    # Efficiency & Safety (20 points total - 10 pts speed, 10 pts safety)
    feedback.append("\n--- Efficiency & Safety ---")
    # Speed score: scales from 10 points (time <= 20s) down to 0 points (time >= 40s)
    if sim_time <= 20.0:
        speed_score = 10.0
    else:
        speed_score = max(0.0, 10.0 - (sim_time - 20.0) / 20.0 * 10.0)
        
    safety_score = 0.0 if crashed else 10.0
    feedback.append(f"  Speed Score (time={sim_time:.1f}s): {speed_score:.2f}/10.0")
    feedback.append(f"  Safety Score (crashed={crashed}): {safety_score:.2f}/10.0")

    # Final Grade Calculation
    base_legs = sum(leg_scores)
    base_turns = sum(turn_scores)
    total_grade = base_legs + base_turns + parking_score + speed_score + safety_score
    
    final_grade_rounded = round(total_grade)

    feedback.append("\n=== Score Arithmetic Breakdown ===")
    feedback.append(f"  Leg Segments (Straightness/Length) : {base_legs:5.2f} / 30.00 pts")
    feedback.append(f"  Corner Turn Angles (90 deg accuracy): {base_turns:5.2f} / 30.00 pts")
    feedback.append(f"  Parking Accuracy (return to start) : {parking_score:5.2f} / 20.00 pts")
    feedback.append(f"  Run Speed Efficiency              : {speed_score:5.2f} / 10.00 pts")
    feedback.append(f"  Safety Bonus (no collision)        : {safety_score:5.2f} / 10.00 pts")
    feedback.append(f"  -------------------------------------------")
    feedback.append(f"  Calculated Grade                   : {total_grade:5.2f} / 100.00 pts")
    feedback.append(f"  GRADE: {final_grade_rounded}%")

    return float(final_grade_rounded), "\n".join(feedback)

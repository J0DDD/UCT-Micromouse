#!/usr/bin/env python3
import os
import sys
import zipfile
import shutil

ALIASES = {
    "milestone0": "milestone0_verification",
    "milestone1": "milestone1_square",
    "milestone2": "milestone2_maze",
}

def build_single_zip(assignment, script_dir, root_dir, assignments_dir, deploy_dir):
    target_assignment_dir = os.path.join(assignments_dir, assignment)
    if not os.path.exists(target_assignment_dir):
        print(f"[Error] Assignment folder not found: {target_assignment_dir}")
        return False
        
    zip_filename = f"{assignment}_autograder.zip"
    zip_path = os.path.join(deploy_dir, zip_filename)
    print(f"\n=== Creating Autograder ZIP: {zip_filename} ===")
    
    files_to_package = [
        # Autograder files
        (os.path.join(script_dir, "setup.sh"), "setup.sh"),
        (os.path.join(script_dir, "run_autograder"), "run_autograder"),
        (os.path.join(script_dir, "grade_runner.py"), "grade_runner.py"),
        
        # Simulator components
        (os.path.join(root_dir, "tools", "physics_sim.py"), "physics_sim.py"),
        (os.path.join(root_dir, "tools", "simulation_config.json"), "simulation_config.json"),
        (os.path.join(root_dir, "python", "micromouse.py"), "micromouse.py"),
        (os.path.join(root_dir, "python", "uct_mouse.py"), "uct_mouse.py"),
        
        # C/Simulink standalone compilation components
        (os.path.join(root_dir, "matlab", "simulink", "PC_client_main.c"), "PC_client_main.c"),
        (os.path.join(root_dir, "firmware", "src", "kernel", "src", "simulink_wrapper.c"), "simulink_wrapper.c"),
        (os.path.join(root_dir, "firmware", "src", "kernel", "inc", "simulink_wrapper.h"), "simulink_wrapper.h"),
    ]
    
    try:
        with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
            # 1. Package explicit files
            for src_path, arc_name in files_to_package:
                if not os.path.exists(src_path):
                    print(f"[Error] Required source file does not exist: {src_path}")
                    return False
                print(f"Adding: {arc_name}")
                zinfo = zipfile.ZipInfo(arc_name)
                zinfo.external_attr = 0o100755 << 16 # unix executable permissions
                with open(src_path, 'rb') as f:
                    zipf.writestr(zinfo, f.read())
                    
            # 2. Package dynamic active_assignment.txt
            print(f"Adding: active_assignment.txt -> {assignment}")
            zipf.writestr("active_assignment.txt", assignment)
            
            # 3. Package all assignments/ folders
            for root, dirs, files in os.walk(assignments_dir):
                for file in files:
                    if file.startswith('.') or file.endswith('.pyc') or '__pycache__' in root:
                        continue
                    full_file_path = os.path.join(root, file)
                    rel_path = os.path.relpath(full_file_path, script_dir)
                    print(f"Adding: {rel_path}")
                    zipf.write(full_file_path, rel_path)
                    
        print(f"[Success] Autograder ZIP created successfully: {zip_path}")
        return True
    except Exception as e:
        print(f"[Error] Failed to build zip archive: {e}")
        return False

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    root_dir = os.path.dirname(os.path.dirname(script_dir))
    assignments_dir = os.path.join(script_dir, "assignments")
    deploy_dir = os.path.join(root_dir, "workspace", "deploy")
    os.makedirs(deploy_dir, exist_ok=True)
    
    target_arg = sys.argv[1].strip().lower() if len(sys.argv) > 1 else "all"
    
    if target_arg in ["all", "everything"]:
        targets = ["milestone0_verification", "milestone1_square", "milestone2_maze"]
    else:
        resolved = ALIASES.get(target_arg, target_arg)
        targets = [resolved]
        
    for t in targets:
        build_single_zip(t, script_dir, root_dir, assignments_dir, deploy_dir)
        
    print("\nAll target autograder ZIPs built in workspace/deploy/.")

if __name__ == "__main__":
    main()

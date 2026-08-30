% =========================================================================
% UCT Micromouse - Unified TCP Autograder Server
% =========================================================================
% This script hosts a TCP server on port 8000. It waits for a student's
% controller (either Python or Simulink Desktop Co-Simulation) to connect.
% Upon connection, it enters a lock-step loop:
%   1. Read {"a":[LeftPWM, RightPWM]} from student.
%   2. Inject PWM into legacy physics Simulink model.
%   3. Step physics model forward by 1 dt.
%   4. Read virtual sensors from physics model.
%   5. Send {"tof_c": X, "lenc": Y, "renc": Z} back to student.
% =========================================================================

function [trajectory_log, run_status] = run_tcp_autograder(physics_model, max_time_s)
    if nargin < 2
        max_time_s = 60.0; % Default 60 seconds
    end
    % Allow overriding the default physics engine
    if nargin < 1
        physics_model = 'mm_simplerobot'; 
    end
    
    trajectory_log = [];
    run_status = 'Completed';
    controller_dt = 0.05; % Default matching the python dashboard 20Hz
    port = 8000;
    
    % 1. Start the TCP Server
    server = tcpserver("127.0.0.1", port);
    configureTerminator(server, "CR/LF");
    disp(['[Autograder] TCP Server listening on localhost:', num2str(port)]);
    disp('[Autograder] Waiting for student code to connect...');
    
    % 2. Wait for Connection
    timeout_s = 60.0;
    t_start = tic;
    while server.Connected == 0
        if toc(t_start) > timeout_s
            error('Timeout waiting for student controller connection on port %d.', port);
        end
        pause(0.1);
    end
    disp('[Autograder] Student connected! Initializing Physics Engine...');
    
    % 3. Initialize the Physics Engine
    % Open the system instead of just loading it so you can watch the canvas
    open_system(physics_model, 'window');
    
    % Zero out the motor inputs immediately so the mouse doesn't drive away 
    % if the .slx file was accidentally saved with non-zero constants!
    set_param([physics_model '/PWM_L_Input'], 'Value', '0');
    set_param([physics_model '/PWM_R_Input'], 'Value', '0');
    
    % Force the simulation to run infinitely so it doesn't stop at t=10.0s
    set_param(physics_model, 'StopTime', 'inf');
    
    % Start the simulation and immediately pause it, ready for stepping
    set_param(physics_model, 'SimulationCommand', 'start');
    set_param(physics_model, 'SimulationCommand', 'pause');
    
    disp('[Autograder] Lock-step simulation running...');
    
    % 4. The Lock-Step Grading Loop
    try
        while server.Connected
            if server.NumBytesAvailable > 0
                % Read incoming JSON packet from student
                rx_str = readline(server);
                
                try
                    data = jsondecode(rx_str);
                    
                    % If the student sends an Actuation packet {"a": [L, R]}
                    if isfield(data, 'a')
                        pwm_l = data.a(1);
                        pwm_r = data.a(2);
                        
                        % --- INJECT INTO PHYSICS ENGINE ---
                        % TODO: Update these paths to point to 'Constant' blocks 
                        % at the input of your legacy physics engine.
                        set_param([physics_model '/PWM_L_Input'], 'Value', num2str(pwm_l));
                        set_param([physics_model '/PWM_R_Input'], 'Value', num2str(pwm_r));
                    end
                    
                    % If the student sends a Configuration packet {"c": {"rate": 100}}
                    if isfield(data, 'c') && isfield(data.c, 'rate')
                        rate_hz = data.c.rate;
                        if rate_hz > 0
                            controller_dt = 1.0 / rate_hz;
                            disp(['[Autograder] Student set control rate to ', num2str(rate_hz), ' Hz (dt=', num2str(controller_dt), 's)']);
                        end
                    end
                    
                    % --- STEP PHYSICS ---
                    % Advance the simulation by controller_dt to maintain perfect virtual lock-step
                    step_str = get_param(physics_model, 'FixedStep');
                    dt = str2double(step_str);
                    if isnan(dt) || dt <= 0
                        dt = 0.01; % Fallback if set to 'auto'
                    end
                    num_steps = max(1, round(controller_dt / dt));
                    
                    for step_idx = 1:num_steps
                        set_param(physics_model, 'SimulationCommand', 'step');
                    end
                    
                    sim_stat = get_param(physics_model, 'SimulationStatus');
                    if strcmp(sim_stat, 'stopped') || strcmp(sim_stat, 'terminating')
                        error('Simulation terminated unexpectedly. Check model for errors.');
                    end
                    
                    % --- CHECK TIME LIMIT ---
                    if get_param(physics_model, 'SimulationTime') >= max_time_s
                        disp(['[Autograder] Time limit of ', num2str(max_time_s), 's reached.']);
                        run_status = 'Time Limit Exceeded';
                        break;
                    end
                    
                    % Force MATLAB to process graphics events (draw the maze figure!)
                    drawnow limitrate;
                    
                    % --- EXTRACT VIRTUAL SENSORS ---
                    % Read the signals entering the Outport blocks in the physics model
                    try
                        rto_tof = get_param([physics_model '/ToF_C_Output'], 'RuntimeObject');
                        
                        if isempty(rto_tof)
                            error('RuntimeObject is empty! Ensure ToF_C_Output is an OUTPORT block, not a Terminator (which is virtual and gets erased).');
                        end
                        
                        % Convert ToF from meters to integer millimeters
                        virtual_tof_c = round(rto_tof.InputPort(1).Data(1) * 1000);
                        
                        rto_lenc = get_param([physics_model '/Enc_L_Output'], 'RuntimeObject');
                        % Scale/round encoders to integers to mock physical ticks
                        virtual_lenc = round(rto_lenc.InputPort(1).Data(1) * 1000);
                        
                        rto_renc = get_param([physics_model '/Enc_R_Output'], 'RuntimeObject');
                        virtual_renc = round(rto_renc.InputPort(1).Data(1) * 1000);
                        
                        % Debug print to ensure time is actually moving forward
                        fprintf('Sim Time: %.3f s | ToF_C: %d mm | L_Enc: %d\n', get_param(physics_model, 'SimulationTime'), virtual_tof_c, virtual_lenc);
                        
                    catch ME_SENSORS
                        disp(['[Warning] Sensor extraction failed: ', ME_SENSORS.message]);
                        virtual_tof_c = 0; virtual_lenc = 0; virtual_renc = 0;
                    end
                    
                    % --- LOG OMNISCIENT TRAJECTORY FOR GRADING ---
                    try
                        rto_x = get_param([physics_model '/True_X_Output'], 'RuntimeObject');
                        rto_y = get_param([physics_model '/True_Y_Output'], 'RuntimeObject');
                        rto_th = get_param([physics_model '/True_Theta_Output'], 'RuntimeObject');
                        
                        if isempty(rto_x) || isempty(rto_y) || isempty(rto_th)
                            error('RuntimeObject is empty for one or more trajectory Outports.');
                        end
                        
                        true_x = rto_x.InputPort(1).Data(1);
                        true_y = rto_y.InputPort(1).Data(1);
                        true_th = rto_th.InputPort(1).Data(1);
                        
                        trajectory_log = [trajectory_log; get_param(physics_model, 'SimulationTime'), true_x, true_y, true_th];
                    catch ME_LOG
                        disp(['[Warning] Trajectory logging failed: ', ME_LOG.message]);
                    end
                    
                    % --- CHECK FOR COLLISIONS ---
                    try
                        rto_col = get_param([physics_model '/Collision_Output'], 'RuntimeObject');
                        if ~isempty(rto_col) && rto_col.InputPort(1).Data(1) > 0.5
                            disp('[Autograder] CRASH! Mouse hit a wall.');
                            run_status = 'Crashed';
                            break;
                        end
                    catch
                        % Silently ignore if collision hook isn't set up yet
                    end
                    
                    % --- SEND TELEMETRY TO STUDENT ---
                    % We format it as a JSON string just like the C-Kernel
                    tx_str = sprintf('{"tof_c":%d, "lenc":%d, "renc":%d}\r\n', ...
                                     virtual_tof_c, virtual_lenc, virtual_renc);
                    write(server, tx_str);
                catch ME_MAIN
                    % Don't silently ignore loop errors! Print them so we can fix them.
                    disp(['[Warning] Main loop error: ', ME_MAIN.message]);
                end
            end
            pause(0.001); % Yield thread to keep MATLAB responsive
        end
    catch ME
        disp(['[Autograder] Simulation stopped: ', ME.message]);
        run_status = 'Error / Crashed';
    end
    
    % 5. Cleanup and prepare for video generation
    set_param(physics_model, 'SimulationCommand', 'stop');
    close_system(physics_model, 0);
    clear server;
    disp('[Autograder] Run complete. Ready to plot trajectory/video.');
end
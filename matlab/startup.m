function startup()
    % STARTUP Configures the MATLAB path and build settings for the UCT-Micromouse project
    
    disp('Initializing UCT-Micromouse MATLAB/Simulink environment...');
    
    % Find the absolute parent repository root based on this script's location
    mfilePath = mfilename('fullpath');
    matlabDir = fileparts(mfilePath);
    projectRoot = fileparts(matlabDir);
    
    % Add essential directories to the MATLAB path
    addpath(fullfile(projectRoot, 'matlab', 'simulink'));
    addpath(fullfile(projectRoot, 'matlab', 'simulator'));
    
    % Configure Simulink cache and code generation directories to redirect build files
    % to the central build/ directory to keep the root directory clean
    try
        Simulink.fileGenControl('set', ...
            'CacheFolder', fullfile(projectRoot, 'build', 'slprj'), ...
            'CodeGenFolder', fullfile(projectRoot, 'build'), ...
            'createDir', true);
        disp('Simulink cache and code-generation folders redirected to build/.');
    catch
        warning('Could not configure Simulink build folders. Ensure Simulink is installed.');
    end
    
    % Configure project models configuration settings for compilation compatibility
    disp('Configuring model build parameters for compilation compatibility...');
    models = {'StudentTemplate', 'StudentTemplate_matlabfunc', 'UCT_KDeploy', 'milestone1_square', 'milestone2_maze'};
    for i = 1:length(models)
        model = models{i};
        try
            if exist(which([model '.slx']), 'file')
                load_system(model);
                changed = false;
                
                % 1. Set GenCodeOnly to 'on' so students don't need MATLAB ARM toolchain registered
                gco = get_param(model, 'GenCodeOnly');
                if ~strcmp(gco, 'on')
                    set_param(model, 'GenCodeOnly', 'on');
                    changed = true;
                end
                
                % 2. Set CodeProfilingInstrumentation to 'off' to prevent parallel loop profiling errors
                cpi = get_param(model, 'CodeProfilingInstrumentation');
                if ~strcmp(cpi, 'off')
                    set_param(model, 'CodeProfilingInstrumentation', 'off');
                    changed = true;
                end
                
                % 3. Link Winsock2 on Windows simulation target
                if ispc
                    libs = get_param(model, 'SimUserLibraries');
                    if contains(libs, 'ws2_32')
                        libs = strrep(libs, 'ws2_32.lib', '');
                        libs = strrep(libs, 'ws2_32', '');
                        libs = strtrim(libs);
                        set_param(model, 'SimUserLibraries', libs);
                        changed = true;
                    end
                    
                    lflags = get_param(model, 'SimCustomLinkerFlags');
                    if ~contains(lflags, 'ws2_32')
                        if isempty(lflags)
                            set_param(model, 'SimCustomLinkerFlags', '-lws2_32');
                        else
                            set_param(model, 'SimCustomLinkerFlags', [lflags ' -lws2_32']);
                        end
                        changed = true;
                    end
                end
                
                if changed
                    save_system(model);
                    fprintf('  Configured and saved build settings for %s.slx\n', model);
                end
            end
        catch ME
            warning('Failed to configure build settings for %s: %s', model, ME.message);
        end
    end
    
    disp('MATLAB path configured successfully.');
    disp('Ready for local TCP/IP Co-Simulation on localhost:8000.');
end

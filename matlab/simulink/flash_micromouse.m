function flash_micromouse(buildInfo)
    disp('=== UCT Micromouse Post-CodeGen Flash Hook ===');
    
    % Get repo root directory (two folders up from matlab/simulink)
    script_path = mfilename('fullpath');
    [simulink_dir, ~, ~] = fileparts(script_path);
    [matlab_dir, ~, ~] = fileparts(simulink_dir);
    [repo_root, ~, ~] = fileparts(matlab_dir);
    
    % Change directory to repo root to execute commands
    original_dir = pwd;
    cd(repo_root);
    
    try
        % Add standard binary directories to PATH environment variable
        % (Needed because GUI-launched MATLAB does not inherit terminal profile paths,
        % and Windows installers often skip adding CMake to the system PATH by default).
        sys_path = getenv('PATH');
        if ispc
            % Windows standard CMake/compilation paths
            paths_to_add = { ...
                'C:\Program Files\CMake\bin', ...
                'C:\Program Files (x86)\CMake\bin', ...
                'C:\msys64\mingw64\bin' ...
            };
            for i = 1:length(paths_to_add)
                if exist(paths_to_add{i}, 'dir')
                    sys_path = [paths_to_add{i} pathsep sys_path];
                end
            end
        else
            % macOS/Linux standard paths
            sys_path = ['/opt/homebrew/bin:/usr/local/bin:/opt/local/bin' pathsep sys_path];
        end
        setenv('PATH', sys_path);
        
        disp('Compiling firmware target with CMake...');
        [status, cmdout] = system('cmake --build firmware/build --target simulink_firmware');
        disp(cmdout);
        if status ~= 0
            warning('UCT_Micromouse:CompileFailed', 'Firmware compilation failed. Please check your C/C++ model code.');
            cd(original_dir);
            return;
        end
        
        disp('Flashing firmware to STM32 board via st-flash...');
        % Try to locate st-flash on standard paths
        st_flash = 'st-flash';
        if exist('/opt/local/bin/st-flash', 'file')
            st_flash = '/opt/local/bin/st-flash';
        elseif exist('/usr/local/bin/st-flash', 'file')
            st_flash = '/usr/local/bin/st-flash';
        end
        
        [status, cmdout] = system([st_flash ' --reset write firmware/binaries/simulink.bin 0x08000000']);
        disp(cmdout);
        if status ~= 0
            warning('UCT_Micromouse:FlashFailed', 'Flashing failed. Ensure the STM32 board is connected and powered ON.');
        else
            disp('Success! Simulink firmware deployed to hardware.');
        end
    catch ME
        cd(original_dir);
        warning('UCT_Micromouse:BuildHookError', 'An unexpected error occurred during build hook: %s', ME.message);
    end
    
    cd(original_dir);
end

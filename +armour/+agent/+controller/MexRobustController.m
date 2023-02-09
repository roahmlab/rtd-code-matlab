classdef MexRobustController < handle
    properties (SetAccess = immutable, GetAccess = private)
        robot_pointer uint64 = 0
        controller_pointer uint64 = 0
    end
    methods
        function self = MexRobustController(model_uncertainty, Kr, alpha, V_max, r_norm_threshold)
            arguments
                model_uncertainty (:,1) double {mustBeNonempty}
                Kr (:,1) double {mustBeNonempty}
                alpha (1,1) double
                V_max (1,1) double
                r_norm_threshold (1,1) double
            end

            % Ensure the mex functions we want exist
            fcn_list = ["createKinovaController", "updateKinovaController", "destroyKinovaController"];
            
            % If any don't exist, compile
            if self.checkMexNotExists(fcn_list)
                disp("Robust controller mex functions don't exist, building!")
                self.compile()
            end

            % Check if we need to do any rebuilds
            if self.checkFileDateTimes(fcn_list)
                disp("Robust controller mex source updated, building!")
                self.compile()
            end

            % Default model
            robot_file = fullfile(basepath(self), "kinova_without_gripper.txt");
            robot_file = char(robot_file);

            % Create the controller
            [self.robot_pointer, self.controller_pointer] = createKinovaController(robot_file, model_uncertainty, Kr, alpha, V_max, r_norm_threshold);

            % Verify
            if ~self.robot_pointer || ~self.controller_pointer
                error("Controller failed to initialize properly!")
            end
        end
        
        % update the controller to get our new control inputs.
        function [u, tau, v] = update(self, q, qd, q_des, qd_des, qdd_des)
            % Verify
            if ~self.robot_pointer || ~self.controller_pointer
                error("Bad class instance!")
            end

            % Get the control inputs
            [u, tau, v] = updateKinovaController(self.robot_pointer, self.controller_pointer, q, qd, q_des, qd_des, qdd_des);
        end

        % Destructer. Sanity check to make sure we don't delete a null
        % pointer.
        function delete(self)
            if self.robot_pointer && self.controller_pointer
                destroyKinovaController(self.robot_pointer, self.controller_pointer);
            end
        end
    end

    methods (Access=private)
        % Verify the mex files exist
        function mex_not_exists = checkMexNotExists(self, fcn_list)
            fcn_list_locations = arrayfun(@which, fcn_list, UniformOutput=false);
            mex_not_exists = any(cellfun(@isempty, fcn_list_locations));

            % Verify that these files are actually in the right place
            if ~mex_not_exists
                classpath = basepath(self);
                valid_path = cellfun(@(path) startsWith(path, classpath, IgnoreCase=true), fcn_list_locations);
                if ~all(valid_path)
                    warning("Expected controller functions are being masked by another function elsewhere! Treating files as nonexisting!")
                    mex_not_exists = true;
                end
            end
        end

        % Check if we need to do a compilation update
        function files_updated = checkFileDateTimes(self, fcn_list)
            % Get the earliest mex'd time
            fcn_list_locations = arrayfun(@which, fcn_list, UniformOutput=false);
            fcn_filedate = cellfun(@dir,fcn_list_locations);
            fcn_filedate = min([fcn_filedate.datenum]);

            % Get the latest source file change
            srcfilepath = fullfile(basepath(self), 'src');
            latest_filedate = dir(srcfilepath);
            % remove '.' and '..'
            latest_filedate = latest_filedate(~ismember({latest_filedate.name},{'.','..'}));
            latest_filedate = max([latest_filedate.datenum]);

            % True if the latest source is later than the earliest mex time
            files_updated = latest_filedate > fcn_filedate;
        end

        % Do a compilation run for all files in this class.
        function compile(self)
            % Extra compilation options as string array
            comp_options = ["CXXFLAGS=$CXXFLAGS -std=c++14 -O2", ""];

            % Get the base path
            currpath = basepath(self);
            mexfilepath = fullfile(currpath, 'private');
            srcfilepath = fullfile(currpath, 'src');

            % Compilation lists
            core_list = ["robot_models.cpp", "rnea.cpp", "robust_controller.cpp"];
            create_file = "createKinovaController";
            update_file = "updateKinovaController";
            destroy_file = "destroyKinovaController";

            % Build compilation commands
            files = [create_file, update_file, destroy_file];
            for file=files
                fprintf("Building %s!\n", file);
                input_files = [file + ".cpp", core_list];
                input_files = fullfile(srcfilepath, input_files);

                % Generate the mex command and run it
                mex_command = ["-outdir", mexfilepath, "-output", file, input_files, comp_options];
                mex_command = num2cell(mex_command);
                mex(mex_command{:});
            end
        end
    end
end

% Utility to retrieve the basepath of the current class file.
function currpath = basepath(obj)
    currfile = which(class(obj));
    [currpath, ~, ~] = fileparts(currfile);
end
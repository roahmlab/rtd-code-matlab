classdef ArmourMexController < armour.agent.ArmourController & rtd.util.MexWrapper & handle
    
    properties (SetAccess = private, GetAccess = private)
        robot_pointer uint64 = 0
        controller_pointer uint64 = 0
    end

    methods
        function reset(self, varargin)
            reset@armour.agent.ArmourController(self, varargin{:})

            % Default model
            robot_file = fullfile(basepath(self), "kinova_without_gripper.txt");
            robot_file = char(robot_file);
            warning('Current robust controller is hardcoded to use kinova_without_gripper!');

            model_uncertainty = 0.03;
            warning('Current robust controller is hardcoded to use model uncertainty of 0.03 for all!');

            % Resize Kr
            Kr = self.Kr;
            if length(self.Kr) == 1
                Kr = self.Kr * ones(self.robot_info.num_q, 1);
            end

            % Create the controller
            self.destroyIfNeeded();
            [self.robot_pointer, self.controller_pointer] = ...
                createKinovaController( ...
                    robot_file, ...
                    model_uncertainty, ...
                    Kr, ...
                    self.alpha_constant, ...
                    self.V_max, ...
                    self.r_norm_threshold ...
                );

            % Verify
            if ~self.robot_pointer || ~self.controller_pointer
                error("Controller failed to initialize properly!")
            end
        end
        
        function [u, info] = getControlInputs(self, t, z_meas)
            % Make sure the controller is valid!
            if ~self.robot_pointer || ~self.controller_pointer
                error("Bad class instance!")
            end

            % Prepare state
            z = z_meas(:);
            position = z(self.robot_state.position_indices);
            velocity = z(self.robot_state.velocity_indices);

            % Prepare trajectory
            trajectory = self.trajectories{end};
            startTime = trajectory.startState.time;
            target = trajectory.getCommand(startTime + t);

            % Get the control inputs from the mex controller
            [u, tau, v] = updateKinovaController( ...
                self.robot_pointer, self.controller_pointer, ...
                position, velocity, ...
                target.position, target.velocity, target.acceleration);
            
            % Extra info output
            info.nominal_input = tau;
            info.robust_input = v;
        end

        % Destructor.
        function delete(self)
            self.destroyIfNeeded();
        end
    end

    methods (Access=private)
        % Sanity check to make sure we don't delete a null pointer.
        function destroyIfNeeded(self)
            if self.robot_pointer && self.controller_pointer
                destroyKinovaController(self.robot_pointer, self.controller_pointer);
            end
            self.robot_pointer = 0;
            self.controller_pointer = 0;
        end
    end

    % Compilation configuration
    methods (Static, Access=protected)
        function fcns = mex_functionsToBuild()
            fcns.createKinovaController = ["createKinovaController.cpp"];
            fcns.updateKinovaController = ["updateKinovaController.cpp"];
            fcns.destroyKinovaController = ["destroyKinovaController.cpp"];
        end

        function [src_folder, common_files, compile_options] = mex_commonCompileInfo()
            src_folder = './src';
            common_files = ["robot_models.cpp", "rnea.cpp", "robust_controller.cpp"];
            compile_options = ["CXXFLAGS=$CXXFLAGS -std=c++14 -O2", ""];
        end

        % Required to pass through private `which` info.
        function out = mex_checkWhich(names)
            out = which(names);
        end
    end

end

% Utility to retrieve the basepath of the current class file.
function currpath = basepath(obj)
    currfile = which(class(obj));
    [currpath, ~, ~] = fileparts(currfile);
end

classdef WaitrMexController < armour.agent.ArmourController
    properties(Access=private)
        mex_controller
    end
    methods
        function self = WaitrMexController(varargin)
            self = self@armour.agent.ArmourController(varargin{:});
            self.mex_controller = armour.agent.ArmourMexController(varargin{:});
        end

        function reset(self, varargin)
            robot_file = fullfile(basepath(self), "kinova_with_gripper.txt");
            self.mex_controller.reset("robot_file", robot_file, varargin{:})

            % Set all variables to match
            self.n_inputs = self.mex_controller.n_inputs;
            self.name = self.mex_controller.name;
            self.Kr = self.mex_controller.Kr;
            self.alpha_constant = self.mex_controller.alpha_constant;
            self.V_max = self.mex_controller.V_max;
            self.r_norm_threshold = self.mex_controller.r_norm_threshold;
            self.ultimate_bound = self.mex_controller.ultimate_bound;
            self.ultimate_bound_position = self.mex_controller.ultimate_bound_position;
            self.ultimate_bound_velocity = self.mex_controller.ultimate_bound_velocity;
            self.trajectories = self.mex_controller.trajectories;
        end

        function setTrajectory(self, trajectory)
            self.mex_controller.setTrajectory(trajectory);
        end

        function [u, info] = getControlInputs(self, varargin)
            [u, info] = self.mex_controller.getControlInputs(varargin{:});
        end

        function out = ultimate_bound_check(self, varargin)
            out = self.mex_controller.ultimate_bound_check(varargin{:});
        end
    end
end

% Utility to retrieve the basepath of the current class file.
function currpath = basepath(obj)
    currfile = which(class(obj));
    [currpath, ~, ~] = fileparts(currfile);
end

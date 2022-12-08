classdef TrajectoryPassthroughController < handle & BaseControllerComponent
properties
    desired_trajectory
    n_outputs = 0 ;
    verbose;
end

methods
%% constructor
%     function self = OpenLoopController(arm_info, arm_state)
%         self.robot_info = arm_info;
%         self.r = arm_state;
%     end
    
%% setup
%     function setup(self,agent)
%         % method: setup(agent)
%         %
%         % Take in an agent instance and set up the relevant parameters
%         % necessary for the low-level controller to operate
%         
%         LLC.n_agent_states = agent.n_states ;
%         LLC.n_agent_inputs = agent.n_inputs ;
%     end
    function setTrajectory(self, trajectory)
        self.desired_trajectory = trajectory;
    end
%% get control inputs
    function U = get_control_inputs(self,t_vec)
        

%% verbose display
    function vdisp(LLC,s,l)
    % Display a string s if the message's verbose level l is greater
    % than or equal to the planner's verbose level.
        if nargin < 3
            l = 1 ;
        end
        if LLC.verbose >= l
            if ischar(s)
                disp(['        LLC: ',s])
            else
                disp('        LLC: String not provided!')
            end
        end
    end
end
end
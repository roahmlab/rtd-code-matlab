classdef ArmourController < BaseControllerComponent & NamedClass & OptionsClass & handle
    
    % Leftover Old Dependencies
    % robot_arm_LLC and uarmtd_robust_CBF_LLC
    % arminfo params by extension
    % arminfo M_min_eigenvalue???
    
    properties
        robot_info ArmourAgentInfo = ArmourAgentInfo.empty()
        robot_state ArmourAgentState = ArmourAgentState.empty()
        LLC_wrapped robot_arm_LLC = uarmtd_robust_CBF_LLC.empty()
        
        n_inputs uint32 = 0
        
        ultimate_bound double
        ultimate_bound_position double
        ultimate_bound_velocity double
        alpha_constant double
        
        trajectories = {}
    end
    
    methods (Static)
        function options = defaultoptions()
            % Configurable options for this component
            options.use_true_params_for_robust = false;
            options.use_disturbance_norm = true;
            options.Kr = 10;
            options.V_max = 3.1e-7;
            options.alpha_constant = 1;
            options.verboseLevel = LogLevel.INFO;
            options.name = '';
        end
    end
    
    methods
        function self = ArmourController(arm_info, arm_state_component, optionsStruct, options)
            arguments
                arm_info ArmourAgentInfo
                arm_state_component ArmourAgentState
                optionsStruct struct = struct()
                options.use_true_params_for_robust
                options.use_disturbance_norm
                options.Kr
                options.V_max
                options.alpha_constant
                options.verboseLevel
                options.name
            end
            self.mergeoptions(optionsStruct, options);
            
            % Set base variables
            self.robot_info = arm_info;
            self.robot_state = arm_state_component;
            self.n_inputs = arm_info.n_links_and_joints;
            
            % Initialize
            self.reset();
        end
        
        function reset(self, optionsStruct, options)
            arguments
                self
                optionsStruct struct = struct()
                options.use_true_params_for_robust
                options.use_disturbance_norm
                options.Kr
                options.V_max
                options.alpha_constant
                options.verboseLevel
                options.name
            end
            options = self.mergeoptions(optionsStruct, options);
            
            % Set up verbose output
            self.name = options.name;
            self.set_vdisplevel(options.verboseLevel);
            
            % Create the LLC with the options
            self.LLC_wrapped = uarmtd_robust_CBF_LLC( ...
                'use_true_params_for_robust', self.options.use_true_params_for_robust, ...
                'use_disturbance_norm', self.options.use_disturbance_norm, ...
                'Kr', self.options.Kr, ...
                'V_max', self.options.V_max, ...
                'alpha_constant', options.alpha_constant);
            
            % Because we want to simplify, recreate the wrapped setup
            % function and merge info out
            % low_level_controller
            self.LLC_wrapped.n_agent_states = self.robot_state.n_states;
            self.LLC_wrapped.n_agent_inputs = self.robot_state.n_inputs;
            % robot_arm_LLC
            self.LLC_wrapped.arm_dimension = self.robot_info.dimension;
            self.LLC_wrapped.arm_n_links_and_joints = self.robot_info.n_links_and_joints;
            self.LLC_wrapped.arm_joint_state_indices = self.robot_state.position_indices;
            self.LLC_wrapped.arm_joint_speed_indices = self.robot_state.velocity_indices;
            % uarmtd_robust_CBF_LLC
            if isprop(self.robot_info, 'M_min_eigenvalue')
                self.LLC_wrapped.ultimate_bound = sqrt(2*self.LLC_wrapped.V_max/self.robot_info.M_min_eigenvalue);
                self.LLC_wrapped.ultimate_bound_position = (1/self.LLC_wrapped.Kr)*self.LLC_wrapped.ultimate_bound;
                self.LLC_wrapped.ultimate_bound_velocity = 2*self.LLC_wrapped.ultimate_bound;
                self.vdisp(sprintf('Computed ultimate bound of %.3f', self.LLC_wrapped.ultimate_bound), LogLevel.GENERAL);
            else
                warning('No minimum eigenvalue of agent mass matrix specified, can not compute ultimate bound');
            end
            
            % flatten out pertinent properties
            self.ultimate_bound = self.LLC_wrapped.ultimate_bound;
            self.ultimate_bound_position = self.LLC_wrapped.ultimate_bound_position;
            self.ultimate_bound_velocity = self.LLC_wrapped.ultimate_bound_velocity;
            self.alpha_constant = self.LLC_wrapped.alpha_constant;
        end
            
        
        function setTrajectory(self, trajectory)
            arguments
                self
                trajectory Trajectory
            end
            self.trajectories = [self.trajectories, {trajectory}];
        end
        
        function [u, tau, v, true_disturbance, true_V, r] = getControlInputs(self, t, z_meas)
            % Prepare trajectory
            trajectory = self.trajectories(end);
            startTime = trajectory.robotState.time;
            
            % Create shims
            planner_info_shim.desired_trajectory = {@(t)unwrap_traj(trajectory.getCommand(startTime + t))};
            A_shim.joint_state_indices = self.robot_state.position_indices;
            A_shim.joint_speed_indices = self.robot_state.velocity_indices;
            A_shim.params = self.robot_info.params;
            A_shim.n_states = self.robot_state.n_states;
            
            % Run the LLC
            if nargout > 3
                [u, tau, v, true_disturbance, true_V, r] = self.LLC_wrapped.get_control_inputs(A_shim, t, z_meas, planner_info_shim);
            else
                [u, tau, v, true_disturbance] = self.LLC_wrapped.get_control_inputs(A_shim, t, z_meas, planner_info_shim);
            end
        end
        
    end
end

% Utility function
function [q, qd, qdd] = unwrap_traj(res)
    q = res.q_des;
    qd = res.q_dot_des;
    qdd = res.q_ddot_des;
end
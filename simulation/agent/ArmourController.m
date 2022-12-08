classdef ArmourController < handle & NamedClass
    
    properties
        arm_info
        arm_state
        LLC_wrapped
        
        ultimate_bound
        ultimate_bound_position
        ultimate_bound_velocity
        alpha_constant
        
        trajectories = {}
    end
    
    methods
        function self = ArmourController(arm_info, arm_state_component, options)
            arguments
                arm_info
                arm_state_component
                options.use_true_params_for_robust = false
                options.use_disturbance_norm = true;
                options.Kr = 10
                options.V_max = 3.1e-7
                options.alpha_constant = 1
            end
            self.arm_info = arm_info;
            self.arm_state = arm_state_component;
            self.LLC_wrapped = uarmtd_robust_CBF_LLC( ...
                'use_true_params_for_robust', options.use_true_params_for_robust, ...
                'use_disturbance_norm', options.use_disturbance_norm, ...
                'Kr', options.Kr, ...
                'V_max', options.V_max, ...
                'alpha_constant', options.alpha_constant);
            
            % Because we want to simplify, recreate the wrapped setup
            % function and merge info out
            % low_level_controller
            self.LLC_wrapped.n_agent_states = self.arm_state.n_states;
            self.LLC_wrapped.n_agent_inputs = self.arm_state.n_inputs;
            % robot_arm_LLC
            self.LLC_wrapped.arm_dimension = self.arm_info.dimension;
            self.LLC_wrapped.arm_n_links_and_joints = self.arm_info.n_links_and_joints;
            self.LLC_wrapped.arm_joint_state_indices = self.arm_info.position_indices;
            self.LLC_wrapped.arm_joint_speed_indices = self.arm_info.velocity_indices;
            % uarmtd_robust_CBF_LLC
            if isprop(self.arm_info, 'M_min_eigenvalue')
                self.LLC_wrapped.ultimate_bound = sqrt(2*self.LLC_wrapped.V_max/self.arm_info.M_min_eigenvalue);
                self.LLC_wrapped.ultimate_bound_position = (1/self.LLC_wrapped.Kr)*self.LLC_wrapped.ultimate_bound;
                self.LLC_wrapped.ultimate_bound_velocity = 2*self.LLC_wrapped.ultimate_bound;
                LLC.vdisp(sprintf('Computed ultimate bound of %.3f', LLC.ultimate_bound), 8);
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
            self.trajectories = [self.trajectories, {trajectory}];
        end
        
        function [u, tau, v, true_disturbance, true_V, r] = getControlInputs(self, t, z_meas)
            planner_info_shim.desired_trajectory = self.trajectories;
            A_shim.joint_state_indices = self.arm_info.position_indices;
            A_shim.joint_speed_indices = self.arm_info.velocity_indices;
            A_shim.params = self.arm_info.params;
            A_shim.n_states = self.arm_state.n_states;
            
            if nargout > 3
                [u, tau, v, true_disturbance, true_V, r] = self.LLC_wrapped.get_control_inputs(A_shim, t, z_meas, planner_info_shim);
            else
                [u, tau, v, true_disturbance] = self.LLC_wrapped.get_control_inputs(A_shim, t, z_meas, planner_info_shim);
            end
        end
        
    end
end
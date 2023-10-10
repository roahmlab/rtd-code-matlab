classdef WaitrDynamics < armour.agent.ArmourDynamics
    % Wrapper for adding force checks
    
    % Extra properties we define
    properties
        u_s = 0.609382421;
        surf_rad = 0.029;
    end
    
    methods (Static)
        function options = defaultoptions()
            options = armour.agent.ArmourDynamics.defaultoptions();
            options.u_s = 0.609382421; 
            options.surf_rad = 0.029;
        end
    end
    
    methods
        function self = WaitrDynamics(arm_info, arm_state_component, controller_component, optionsStruct, options)
            arguments
                arm_info armour.agent.ArmourAgentInfo
                arm_state_component armour.agent.ArmourAgentState
                controller_component armour.agent.ArmourController
                optionsStruct struct = struct()
                options.time_discretization
                options.measurement_noise_points
                options.measurement_noise_pos_sigma
                options.measurement_noise_vel_sigma
                options.log_controller
                options.verboseLevel
                options.name
                options.u_s
                options.surf_rad
            end
            self@armour.agent.ArmourDynamics(arm_info, arm_state_component, controller_component);
            self.mergeoptions(optionsStruct, options);
            
            self.reset()
        end
        
        function reset(self, optionsStruct, options)
            arguments
                self
                optionsStruct struct = struct()
                options.time_discretization
                options.measurement_noise_points
                options.measurement_noise_pos_sigma
                options.measurement_noise_vel_sigma
                options.log_controller
                options.verboseLevel
                options.name
                options.u_s
                options.surf_rad
            end
            options = self.mergeoptions(optionsStruct, options);
            reset@armour.agent.ArmourDynamics(self);

            % Grasp constraint parameters
            self.u_s = options.u_s;
            self.surf_rad = options.surf_rad;
        end

        % Check that the grasp constraints aren't violated
        function out = grasp_constraint_check(self, t_check_step)
            % retrieve the last log entry
            entries = self.controller_log.get('input_time', 'input', 'z', flatten=false);
            t_input = entries.input_time{end};
            z = entries.z{end};
            
            % interpolate for the t_check_step and get agent input
            % trajectory interpolated to time
            t_check = t_input(1):t_check_step:t_input(end);
            z_check = interp1(t_input, z.', t_check).';
            
            % run grasp check
            self.vdisp('Running grasp checks!', 'INFO');
            out = false;
            sep_val = false; % optimism!
            slip_val = false; % more optimism!!
            tip_val = false; % even more optimism!!!
            t_idx = 1;

            while ~sep_val && ~slip_val && ~tip_val && t_idx <= length(t_check)
                out = false;
                sep_val = false;
                slip_val = false;
                tip_val = false;

                t = t_check(t_idx);
                z = z_check(:,t_idx);
                q = z(self.robot_state.position_indices);
                qd = z(self.robot_state.velocity_indices);

                % True dynamics
                [M, C, g] = self.calculate_dynamics(q, qd, self.robot_info.params.true);
    
                for i = 1:length(q)
                    M(i,i) = M(i,i) + self.robot_info.transmission_inertia(i);
                end

                u = self.controller.getControlInputs(t, z);
    
                % calculate acceleration 
                qdd = M\(u-C*qd-g);
                
                %% Calculating forces
                % call RNEA on current configuration (assuming at rest? cannot)
                % (also assuming use_gravity is true)
                [tau, f, n] = armour.legacy.dynamics.rnea(q, qd, qd, qdd, true, self.robot_info.params.true);

                fx = f(1,10);
                fy = f(2,10);
                fz = f(3,10);

                %% Calculating Constraints (written as <0)
                sep = -1*fz; %fz; %
%                 slip = sqrt(fx^2+fy^2) - self.u_s*abs(fz);
                slip2 = fx^2+fy^2 - self.u_s^2*fz^2;
%                 ZMP = cross([0;0;1],n(:,10))./dot([0;0;1],f(:,10));
%                 tip = sqrt(ZMP(1)^2 + ZMP(2)^2) - self.surf_rad; % + tip_threshold;
                ZMP_top = cross([0;0;1],n(:,10));
                ZMP_bottom = dot([0;0;1],f(:,10));
                tip2 = ZMP_top(1)^2 + ZMP_top(2)^2 - self.surf_rad^2*ZMP_bottom^2;

                if (sep > 1e-6) || (slip2 > 1e-6) || (tip2 > 1e-6)
                    out = true;
                end
                
                if sep > 1e-6
                    sep_val = true;
                end
                
                if slip2 > 1e-6
                    slip_val = true;
                end
                
                if tip2 > 1e-6
                    tip_val = true;
                end

                t_idx = t_idx + 1;

            end

            if out
                if sep_val
                    self.vdisp(['Grasp separation violation detected at t = ',num2str(t_check(t_idx))], 'ERROR');
                end
                if slip_val
                    self.vdisp(['Grasp slipping violation detected at t = ',num2str(t_check(t_idx))], 'ERROR');
                end
                if tip_val
                    self.vdisp(['Grasp tipping violation detected at t = ',num2str(t_check(t_idx))], 'ERROR');
                end
            else
                self.vdisp('No grasp violations detected', 'INFO');
            end
        end
    end
end
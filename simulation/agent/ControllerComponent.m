classdef ControllerComponent < handle
    %ControllerComponent
    % TODO Organize & Complete
    % Merge LLC or just adapt for now? Probably adapt
    
    properties
        % General information of the robot arm
        arm_info RoboticsToolboxArmRobotInfo
        
        arm_state_component
        
        LLC
    end
    
    methods
        function self = ControllerComponent(arm_info,arm_state_component)
            self.arm_info = arm_info;
            self.arm_state_component = arm_state_component;
        end
        %% agent
        function [T,U,Z] = move_setup(A,t_move,T_ref,U_ref,Z_ref)
            % method: [T,U,Z] = move_setup(A,t_move,T_ref,U_ref,Z_ref)
            %
            % Given an amount of time t_move for the agent to move, and a
            % reference time vector, output a time vector to actually use
            % during the call to move the agent.
            
            % make sure the reference time is a row vector
            T_ref = T_ref(:)' ;
            
            % make sure the reference time is unique, and only get those
            % rows of U_ref and Z_ref
            [T_ref,unique_idxs,~] = unique(T_ref,'stable') ;
            U_ref = U_ref(:,unique_idxs) ;
            if ~isempty(Z_ref)
                Z_ref = Z_ref(:,unique_idxs);
            end
            
            % sanity check the timing
            if T_ref(1) > 0
                warning(['Provided reference time does not start from 0! ',...
                    'The reference trajectory will be padded with the ',...
                    'first reference input and reference state at time 0'])
                T_ref = [0, T_ref(:)'] ;
                U_ref = [U_ref(:,1), U_ref] ;
                Z_ref = [Z_ref(:,1), Z_ref] ;
            end
            
            if T_ref(end) < t_move
                warning(['Provided input time vector is shorter than the ',...
                    'desired motion time! The agent will still be ',...
                    'moved for the duration t_move. The reference ',...
                    'time, input, and trajectory will be padded ',...
                    'to be of duration t_move.'])
                
                T_ref = [T_ref(:)', t_move] ;
                U_ref = [U_ref, U_ref(:,end)] ;
                Z_ref = [Z_ref, Z_ref(:,end)] ;
            end
            
            % get the amount of time to actually move the agent
            T_log = T_ref < t_move ;
            
            % make sure t_move itself is included in the time vector (this
            % solves a nasty bug that could introduce NaNs in the agent's
            % state unintentionally!)
            T = [T_ref(T_log), t_move] ;
            
            % interpolate the reference input and trajectory to pass to the
            % agent's move method
            if nargin < 5 || isempty(Z_ref)
                U = match_trajectories(T,T_ref,U_ref) ;
                Z = [] ;
            else
                [U,Z] = match_trajectories(T,T_ref,U_ref,T_ref,Z_ref) ;
            end
        end
        %% ARMOUR
        function out = input_check(A, t_start)
            % create time vector for checking
            t_agent = A.time(end);
            t_check = t_start:A.traj_check_time_discretization:t_agent;

            if isempty(t_check) || t_check(end) ~= t_agent
                t_check = [t_check, t_agent] ;
            end

            % get agent input trajectory interpolated to time
            u_agent = match_trajectories(t_check,A.time,A.input) ;

            % check torque bounds
            A.vdisp('Running input check!',3);
            out = false;
            for t_idx = 1:length(t_check)
                u = u_agent(:, t_idx);
                for i = 1:length(u)
                    if u(i) > A.joint_input_limits(2, i) || u(i) < A.joint_input_limits(1, i)
                        fprintf('Time %.2f, joint %d torque exceeded: %.2f vs +-%.2f \n', t_check(t_idx), i, u(i), A.joint_input_limits(2, i));
                        out = true;
                    end
                end
            end
            
            if ~out
                A.vdisp('No inputs exceeded', 3);
            end
        end
    end    
end


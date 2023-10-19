classdef ArmRobotStateInstance < rtd.entity.states.BaseEntityStateInstance
% Single atomic state instance for an generic arm robot with position, velocity, and acceleration.
%
% This is a concrete implementation of the BaseEntityStateInstance class.
% It is used to represent the state of a generic arm robot with position,
% velocity, and acceleration.
%
% --- More Info ---
% Author: Adam Li (adamli@umich.edu)
% Written: 2023-09-12
%
% See also: rtd.entity.states.BaseEntityStateInstance
%
% --- More Info ---
%

    properties
        % Position of each joint in an arm robot in configuration space
        position(:,1) double

        % Velocity of each joint in an arm robot in configuration space
        velocity(:,1) double

        % Acceleration of each joint in an arm robot in configuration space
        acceleration(:,1) double
    end

    properties (Dependent)
        % number of joints in the arm robot, based on the size of the position vector
        num_joints

        % expected size of a state space which captures this state instance
        num_states
    end

    % methods that must be implemented
    methods
        function state = getStateSpace(self, options)
            % Get the state space of the robot state instance
            %
            % Returns a matrix of the state space of the robot. The rows
            % correspond to the state space indices provided in the options
            % struct. The columns correspond to the individual robot state
            % instances.
            %
            % Arguments:
            %   options.position_idxs (uint32) - desired indices of the position states
            %   options.velocity_idxs (uint32) - desired indices of the velocity states
            %   options.acceleration_idxs (uint32) - desired indices of the acceleration states
            %
            % Returns:
            %   state (double) - matrix of the state space of the robot for the state instances
            %
            arguments
                self(1,:) rtd.entity.states.ArmRobotStateInstance
                options.position_idxs(1,:) uint32 = []
                options.velocity_idxs(1,:) uint32 = []
                options.acceleration_idxs(1,:) uint32 = []
            end

            [pos_idx, vel_idx, acc_idx] = self(1).resolveStateSpaceIndices( ...
                options.position_idxs, options.velocity_idxs, options.acceleration_idxs);
            state = zeros(self(1).num_states, length(self));
            state(pos_idx, :) = [self.position];
            state(vel_idx, :) = [self.velocity];
            state(acc_idx, :) = [self.acceleration];
        end

        function setStateSpace(self, state_data, options)
            % Set the state space of the robot state instance
            %
            % Sets the state space of the robot. The rows correspond to the
            % state space indices provided in the options struct. The columns
            % correspond to the individual robot state instances.
            %
            % Arguments:
            %   state_data (double) - matrix of the state space of the robot for the state instances
            %   options.position_idxs (uint32) - indices of the position states in state_data
            %   options.velocity_idxs (uint32) - indices of the velocity states in state_data
            %   options.acceleration_idxs (uint32) - indices of the acceleration states in state_data
            %
            arguments
                self(1,:) rtd.entity.states.ArmRobotStateInstance
                state_data(:,:) double
                options.position_idxs(1,:) uint32 = []
                options.velocity_idxs(1,:) uint32 = []
                options.acceleration_idxs(1,:) uint32 = []
            end

            pos_idx = options.position_idxs;
            vel_idx = options.velocity_idxs;
            acc_idx = options.acceleration_idxs;

            pos_data_to_deal = num2cell(state_data(pos_idx, :), 1);
            vel_data_to_deal = num2cell(state_data(vel_idx, :), 1);
            acc_data_to_deal = num2cell(state_data(acc_idx, :), 1);
            [self.position] = deal(pos_data_to_deal{:});
            [self.velocity] = deal(vel_data_to_deal{:});
            [self.acceleration] = deal(acc_data_to_deal{:});
        end
    end

    % helper methods internal to the class
    methods (Access=private)
        function [pos_idx, vel_idx, acc_idx] = resolveStateSpaceIndices(self, pos_idx, vel_idx, acc_idx)
            % resolveStateSpaceIndices - Resolve the state space indices
            %   Resolves the state space indices for the position, velocity,
            %   and acceleration. If the indices are not provided, then they
            %   are automatically resolved. If they are provided, then they
            %   must be valid and match the number of states in the state
            %   space.
            bad_pos_idx = length(pos_idx) ~= length(self.position);
            bad_vel_idx = length(vel_idx) ~= length(self.velocity);
            bad_acc_idx = length(acc_idx) ~= length(self.acceleration);

            need_pos_idx = isempty(pos_idx) && ~isempty(self.position);
            need_vel_idx = isempty(vel_idx) && ~isempty(self.velocity);
            need_acc_idx = isempty(acc_idx) && ~isempty(self.acceleration);
            
            bad_count = bad_pos_idx + bad_vel_idx + bad_acc_idx;
            need_count = need_pos_idx + need_vel_idx + need_acc_idx;

            % Short circuit if all is good
            if (bad_count + need_count) == 0
                return
            end

            % Otherwise resolve some indices
            if isempty(pos_idx) && isempty(vel_idx) && isempty(acc_idx)
                pos_idx = 1:length(self.position);
                vel_idx = (1:length(self.velocity)) + pos_idx(end);
                acc_idx = (1:length(self.acceleration)) + vel_idx(end);
                
            elseif bad_count ~= need_count
                errMsg = MException('ArmRobotStateInstance:BadStateIndices', ...
                    'Invalid / Unmatching state indices provided for state space');
                throw(errMsg)

            elseif need_count > 1
                errMsg = MException('ArmRobotStateInstance:BadStateIndices', ...
                    'Must provide enough indices for valid state space');
                throw(errMsg)

            elseif need_count
                idx_arr = 1:self.num_states;
                % Delete already known indices
                idx_arr(pos_idx) = [];
                idx_arr(vel_idx) = [];
                idx_arr(acc_idx) = [];
                
                if need_pos_idx
                    pos_idx = idx_arr;
                elseif need_vel_idx
                    vel_idx = idx_arr;
                else
                    acc_idx = idx_arr;
                end
            end
        end
    end

    % methods for the dependent properties
    methods
        function n_joints = get.num_joints(self)
            n_joints = length(self.position);
        end

        function n_states = get.num_states(self)
            n_states = length(self.position) + length(self.velocity) + length(self.acceleration);
        end
    end
end
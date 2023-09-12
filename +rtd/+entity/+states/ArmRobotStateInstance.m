classdef ArmRobotStateInstance < rtd.entity.states.BaseEntityStateInstance
    % ArmRobotState
    % Information on the atomic state of the robot at a given point of
    % time. Each hard instance of this (unique object, not seperate
    % handles to the same underlying object) will have a unique uuid.
    properties
        position(:,1) double
        velocity(:,1) double
        acceleration(:,1) double
    end
    properties (Dependent)
        num_joints
        num_states
    end

    methods
        function state = getStateSpace(self, options)
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

    methods
        function n_joints = get.num_joints(self)
            n_joints = length(self.position);
        end

        function n_states = get.num_states(self)
            n_states = length(self.position) + length(self.velocity) + length(self.acceleration);
        end
    end

    methods (Access=private)
        function [pos_idx, vel_idx, acc_idx] = resolveStateSpaceIndices(self, pos_idx, vel_idx, acc_idx)
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
end
classdef GenericEntityStateInstance < rtd.entity.states.BaseEntityStateInstance
% Single atomic state instance for an generic entity with some abitrary state
%
% This is a concrete implementation of the BaseEntityStateInstance class.
% It is used to represent a single atomic state instance for an generic
% entity with some abitrary state.
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
        % State data for the entity.
        % This could be xyz position, quaternion, etc.
        state_data(:,1) double
    end

    properties (Dependent)
        % Number of states in the entity
        num_states
    end

    % methods that must be implemented
    methods
        function state = getStateSpace(self)
            % Get the state space of the entity
            %
            % This method returns the state space of the entity as a matrix.
            % Each column of the matrix represents a single state instance.
            %
            % Returns:
            %   state (double): state space of the entity
            %
            arguments
                self(1,:) rtd.entity.states.GenericEntityStateInstance
            end
            
            state = [self.state_data];
        end

        function setStateSpace(self, state_data)
            % Set the state space of the entity
            %
            % This method sets the state space of the entity from a matrix.
            % Each column of the matrix represents a single state instance.
            %
            % Parameters:
            %   state_data (double): state space of the entity
            %
            arguments
                self(1,:) rtd.entity.states.GenericEntityStateInstance
                state_data(:,:) double
            end

            state_data_to_deal = num2cell(state_data,1);
            [self.state_data] = deal(state_data_to_deal{:});
        end
    end

    % methods for dependent properties
    methods
        function n_states = get.num_states(self)
            n_states = length(self.state_data);
        end
    end
end
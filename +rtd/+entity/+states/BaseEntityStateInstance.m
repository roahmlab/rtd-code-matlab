classdef BaseEntityStateInstance < rtd.util.mixins.UUID & handle
% BaseEntityStateInstance is the base class for all state instances
%
% This represents the state of some entity at a point in time. Each hard
% instance of this (unique object, not seperate handles to the same
% underlying object) will have a unique uuid.
%
% State in this case refers to configuration states for the entity, and
% may not correspond to the physical position of the entity. For example,
% this may store information about the extension of a robotic planar platform,
% which would be used in combination with a kinematic model to determine
% the actual position of the platform.
%
% This class is abstract, and should not be instantiated directly. Instead,
% use one of the subclasses.
%
% --- More Info ---
% Author: Adam Li (adamli@umich.edu)
% Written: 2022-08-24
% Last Revised: 2023-09-12 (Adam Li)
%
% --- Revision History ---
% 2023-09-12 - Added getTimes and setTimes. Time is forcefully set to a single value
% 2023-09-08 - Removed state as a required property, and added getStateSpace / setStateSpace interface
%
% --- More Info ---
%

    properties
        % time is a scalar double, and is the time at which this state
        % instance was recorded
        time(1,1) double
    end
    
    properties (Abstract, Dependent)
        % num_states is the number of states in the state-space form
        num_states
    end
    
    % Interface methods that must be implemented by subclasses
    methods (Abstract)
        % getStateSpace returns the state-space form of a vector of state
        % instance, in the form of a matrix of size num_states x num_time
        % where num_time is the number of time points in the state instance
        state = getStateSpace(self, options)

        % setStateSpace sets the state-space form of a vector of state instance,
        % in the form of a matrix of size num_states x num_time
        % where num_time is the number of time points in the state instance
        setStateSpace(self, state, options)
    end

    % Base methods for time from an array of BaseEntityStateInstances
    methods
        function times = getTimes(self)
            % get times for an array of BaseEntityStateInstances
            %
            % getTimes returns the times at which the vector elements of state
            % instance was recorded, as a vector of size 1 x num_time
            %
            % Returns:
            %   times (double): times at which the state instance was recorded
            %
            arguments
                self(1,:) rtd.entity.states.BaseEntityStateInstance
            end
            
            times = [self.time];
        end

        function setTimes(self, time_data)
            % set times for an array of BaseEntityStateInstances
            %
            % setTimes sets the times at which the vector elements of state
            % instance was recorded, as a vector of size 1 x num_time
            %
            % Arguments:
            %   time_data (double): times at which the state instance was recorded
            %
            arguments
                self(1,:) rtd.entity.states.BaseEntityStateInstance
                time_data(1,:) double
            end

            time_data_to_deal = num2cell(time_data,1);
            [self.time] = deal(time_data_to_deal{:});
        end
    end
end
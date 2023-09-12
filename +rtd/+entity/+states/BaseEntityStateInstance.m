classdef BaseEntityStateInstance < rtd.util.mixins.UUID & handle
    % RobotState
    % The state of a generic robot at a point in time, which is saved.
    % Each hard instance of this (unique object, not seperate handles to
    % the same underlying object) will have a unique uuid.
    %
    % Update Sept 8, 2023. state as a required property was removed, and an
    % interface to retrieve the state-space form / set via some state-space
    % form is now implemented.
    % This way you can get the statespace in a desired form, or set the
    % values from said desired form, but the underlying storage becomes
    % more meaningful
    % Time is also forcefully set to a single value
    properties
        time(1,1) double
    end
    properties (Abstract, Dependent)
        num_states
    end
    methods (Abstract)
        state = getStateSpace(self, options)
        setStateSpace(self, state, options)
    end
    methods
        function times = getTimes(self)
            arguments
                self(1,:) rtd.entity.states.BaseEntityStateInstance
            end
            times = [self.time];
        end
        function setTimes(self, time_data)
            arguments
                self(1,:) rtd.entity.states.BaseEntityStateInstance
                time_data(1,:) double
            end
            time_data_to_deal = num2cell(time_data,1);
            [self.time] = deal(time_data_to_deal{:});
        end
    end
end
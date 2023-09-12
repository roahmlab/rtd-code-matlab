classdef GenericEntityStateInstance < rtd.entity.states.BaseEntityStateInstance
    % RobotState
    properties
        state_data(:,1) double
    end
    properties (Dependent)
        num_states
    end
    methods
        function state = getStateSpace(self)
            arguments
                self(1,:) rtd.entity.states.GenericEntityStateInstance
            end
            state = [self.state_data];
        end
        function setStateSpace(self, state_data)
            arguments
                self(1,:) rtd.entity.states.GenericEntityStateInstance
                state_data(:,:) double
            end
            state_data_to_deal = num2cell(state_data,1);
            [self.state_data] = deal(state_data_to_deal{:});
        end
        function n_states = get.num_states(self)
            n_states = length(self.state_data);
        end
    end
end
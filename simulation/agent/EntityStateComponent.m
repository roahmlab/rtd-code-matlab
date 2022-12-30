classdef EntityStateComponent < handle
    % EntityState Interface for some Entity state
    properties (Abstract)
        entity_info EntityInfo
        % state space representation
        n_states(1,1) uint32
        state double
        % this might not remain here
        time(1,:) double
    end
    methods (Abstract)
        reset(self)
        state = get_state(self, time)
        random_init(self)
        commit_state_data(self, times, states)
    end
end
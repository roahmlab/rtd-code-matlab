classdef ReachableSets < handle
    % ReachableSets
    % This either encapsulates the reachable sets in memory, or enables the
    % online computation of reachable sets. It acts as a generator for a
    % single instance of ReachableSet
    methods (Abstract)
        % An example constructor, but can take anything needed
        %self = ReachableSets( ...
        %            robotInfo ...
        %        )
        
        % Obtains the relevant reachable set for the robotstate provided
        % and outputs the singular instance of a reachable set.
        % Returns ReachbleSet
        reachableSet = getReachableSet(self, robotState)
    end
end
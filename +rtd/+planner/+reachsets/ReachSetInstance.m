classdef ReachSetInstance < rtd.util.mixins.UUID & handle
    % ReachSetInstance
    % This is just an individual instance of a reachable set. It should
    % hold the necessary information to make a nonlinear constraint. If a
    % generated nonlinear constraint function is not atomic, in that it may
    % change class properties, the cache for the generating RS class should
    % be disabled by setting cache_max_size to 0.
    properties (Abstract)
        parameter_range
        output_range
    end
    methods (Abstract)
        % An example constructor, but can take anything needed for the
        % respective ReachableSets class.
        %self = ReachableSets( ...
        %            robotInfo ...
        %        )
        
        % Handles the obstacle-frs pair or similar to generate the
        % nlconstraint.
        % Returns a function handle for the nlconstraint generated
        % where the function's return type is [c, ceq, gc, gceq]
        nlconFunction = genNLConstraint(self, worldState)
    end
end
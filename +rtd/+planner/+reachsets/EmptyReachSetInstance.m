classdef EmptyReachSetInstance < rtd.util.mixins.UUID & matlab.mixin.Heterogeneous & handle
    % ReachSetInstance
    % This is just an individual instance of a reachable set. It should
    % hold the necessary information to make a nonlinear constraint. If a
    % generated nonlinear constraint function is not atomic, in that it may
    % change class properties, the cache for the generating RS class should
    % be disabled by setting cache_max_size to 0.
    properties
        parameter_range
        output_range
    end
    methods
        % An example constructor, but can take anything needed for the
        % respective ReachableSets class.
        %self = ReachableSets( ...
        %            robotInfo ...
        %        )
        
        % Handles the obstacle-frs pair or similar to generate the
        % nlconstraint.
        % Returns a function handle for the nlconstraint generated
        % where the function's return type is [c, ceq, gc, gceq]
        function nlconFunction = genNLConstraint(self, worldState)
            nlconFunction = @NOP;
        end
    end
end

function [h, heq, grad_h, grad_heq] = NOP(varargin)
    h = [];
    heq = [];
    grad_h = [];
    grad_heq = [];
end

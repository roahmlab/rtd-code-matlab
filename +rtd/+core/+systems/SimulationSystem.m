classdef SimulationSystem < rtd.core.mixins.UUID & handle
    properties (Abstract)
        time (1,:) double
        time_discretization (1,1) double
        system_log rtd.core.containers.VarLogger
    end
    methods (Abstract)
        reset(self)
        %check = pre_checks(self)
        %results = update(self, t_move)
        %check = post_checks(self)
    end
end

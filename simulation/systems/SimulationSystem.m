classdef SimulationSystem < mixins.UUIDbase & handle
    properties (Abstract)
        time (1,:) double
        time_discretization (1,1) double
        system_log VarLogger
    end
    methods (Abstract)
        reset(self)
        %check = pre_checks(self)
        %results = update(self, t_move)
        %check = post_checks(self)
    end
end

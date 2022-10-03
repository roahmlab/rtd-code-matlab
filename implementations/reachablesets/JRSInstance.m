classdef JRSInstance < ReachableSetInstance
    % JRSInstance
    % This is just an individual instance of an original ARMTD JRS.
    properties
        parameter_range = [-1.0, 1.0]
        output_range = [-1.0, 1.0]
        n_k = []
        
        % properties carried over from the original implementation
        q_des
        dq_des
        ddq_des
        q
        dq
        dq_a
        ddq_a
        R_des
        R_t_des
        R
        R_t
        jrs_info
        
        % New properties to flatten the structure
        n_q
        n_t
        k_id
    end
    methods
        % An example constructor, but can take anything needed for the
        % respective ReachableSets class.
        %self = ReachableSets( ...
        %            robotInfo ...
        %        )
        function initialize(self)
            % expand the parameter range to match the number of joints
            self.parameter_range = ones(self.jrs_info.n_k, 1) * self.parameter_range;
            self.output_range = self.jrs_info.c_k + [-1.0, 1.0] .* self.jrs_info.g_k;
            
            self.n_q = self.jrs_info.n_q;
            self.n_k = self.jrs_info.n_k;
            self.n_t = self.jrs_info.n_t;
            self.k_id = self.jrs_info.k_id;
        end
        
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
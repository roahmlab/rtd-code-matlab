classdef JRSInstance < rtd.planner.reachsets.ReachSetInstance
    % JRSInstance
    % This is just an individual instance of an original ARMTD JRS.
    properties
        input_range = [-1.0, 1.0]
        output_range = [-1.0, 1.0]
        num_parameters = 0
        
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
        n_k
    end
    methods
        % An example constructor, but can take anything needed for the
        % respective ReachableSets class.
        %self = ReachableSets( ...
        %            robotInfo ...
        %        )
        function initialize(self, traj_type)
            % expand the parameter range to match the number of joints
            switch traj_type
                case 'orig'
                    c_k = self.jrs_info.c_k;
                    g_k = self.jrs_info.g_k;
                case 'bernstein'
                    c_k = self.jrs_info.c_k_bernstein;
                    g_k = self.jrs_info.g_k_bernstein;
            end
            self.input_range = ones(self.jrs_info.n_k, 1) * self.input_range;
            self.output_range = c_k + [-1.0, 1.0] .* g_k;
            
            self.n_q = self.jrs_info.n_q;
            self.num_parameters = self.jrs_info.n_k;
            self.n_k = self.jrs_info.n_k;
            self.n_t = self.jrs_info.n_t;
            self.k_id = self.jrs_info.k_id;
        end
        
        % Handles the obstacle-frs pair or similar to generate the
        % nlconstraint.
        % Returns a function handle for the nlconstraint generated
        % where the function's return type is [c, ceq, gc, gceq]
        function nlconFunction = genNLConstraint(self, worldState)
            nlconFunction = [];
        end
    end
end
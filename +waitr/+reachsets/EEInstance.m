classdef EEInstance < rtd.planner.reachsets.ReachSetInstance
    % EEInstance
    % This is just an individual instance of an original ARMTD EE.
    properties
        input_range = [-1.0, 1.0]
        output_range = []
        num_parameters = 0
        
        % properties carried over from the original implementation
        sep_poly
        slip_poly
        tip_poly
        n_q
        n_t
    end
    methods
        % An example constructor, but can take anything needed for the
        % respective ReachableSets class.
        function self = EEInstance( ...
                    sep_poly, slip_poly, tip_poly, jrsInstance ...
                )
            self.sep_poly = sep_poly;
            self.slip_poly = slip_poly;
            self.tip_poly = tip_poly;
            self.n_q = jrsInstance.n_q;
            self.n_t = jrsInstance.n_t;
            self.num_parameters = jrsInstance.num_parameters;
            
            self.input_range = jrsInstance.input_range;
            self.output_range = [];
        end
        
        % Handles the obstacle-frs pair or similar to generate the
        % nlconstraint.
        % Returns a function handle for the nlconstraint generated
        % where the function's return type is [c, ceq, gc, gceq]
        function nlconFunction = genNLConstraint(self, worldState)
            constraints = {}; % cell will contain functions of $k$ for evaluating constraints
            grad_constraints = {}; % cell will contain functions of $k$ for evaluating gradients
            sep_poly = self.sep_poly;
            slip_poly = self.slip_poly;
            tip_poly = self.tip_poly;
            % add the grasp constraints here
            for i = 1:self.n_t
                % adding separation constraints
                sep_int = interval(sep_poly{i,1});
%                 fprintf('ADDED GRASP SEPARATION CONSTRAINT \n')
                constraints{end+1,1} = @(k) slice(sep_poly{i,1},k);
                grad_sep_poly = grad(sep_poly{i,1},self.num_parameters);
                grad_constraints{end+1, 1} = @(k) cellfun(@(C) slice(C, k), grad_sep_poly);
                
                % adding slipping constraints
                slip_int = interval(slip_poly{i,1});
%                 fprintf('ADDED GRASP SLIPPING CONSTRAINT \n')
                constraints{end+1,1} = @(k) slice(slip_poly{i,1},k);
                grad_slip_poly = grad(slip_poly{i,1},self.num_parameters);
                grad_constraints{end+1, 1} = @(k) cellfun(@(C) slice(C, k), grad_slip_poly);
                
                % adding tipping constraints
                tip_int = interval(tip_poly{i,1});
%                 fprintf('ADDED GRASP TIPPING CONSTRAINT \n')
                constraints{end+1,1} = @(k) slice(tip_poly{i,1},k);
                grad_tip_poly = grad(tip_poly{i,1},self.num_parameters);
                grad_constraints{end+1, 1} = @(k) cellfun(@(C) slice(C, k), grad_tip_poly);
            end
            nlconFunction = @(k) eval_constraints(k, length(constraints), constraints, grad_constraints);
        end
    end
end

% NOTE: remember that smooth constraints still need to be considered.
function [h, heq, grad_h, grad_heq] = eval_constraints(k, n_c, constraints, grad_constraints)
    h = zeros(n_c, 1);
    grad_h = zeros(length(k), n_c);

    for i = 1:n_c
        h(i) = constraints{i}(k);
        grad_h(:, i) = grad_constraints{i}(k);
    end

    grad_heq = [];
    heq = [];
end

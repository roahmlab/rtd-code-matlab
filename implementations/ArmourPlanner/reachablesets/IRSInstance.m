classdef IRSInstance < ReachableSetInstance & rtd.util.mixins.NamedClass
    % IRSInstance
    % This is just an individual instance of input reachable set from
    % armour.
    properties
        parameter_range = [-1.0, 1.0]
        output_range = []
        n_k = []
        
        % properties carried over from the original implementation
        u_ub
        u_lb
        n_q
        n_t
    end
    methods
        function self = IRSInstance( ...
                    u_ub, u_lb, jrsInstance ...
                )
            self.u_ub = u_ub;
            self.u_lb = u_lb;
            self.n_q = jrsInstance.n_q;
            self.n_t = jrsInstance.n_t;
            self.n_k = jrsInstance.n_k;
            
            self.parameter_range = jrsInstance.parameter_range;
            self.output_range = [];
        end
        
        % Generates an nlconstraint if needed, or will return a NOP
        % function.
        % Returns a function handle for the nlconstraint generated
        % where the function's return type is [c, ceq, gc, gceq]
        function nlconFunction = genNLConstraint(self, worldState)
            constraints = {}; % cell will contain functions of $k$ for evaluating constraints
            grad_constraints = {}; % cell will contain functions of $k$ for evaluating gradients
            for i = 1:self.n_t
                for j = 1:self.n_q

                    % first check if constraints are necessary, then add
                    u_ub_int = interval(self.u_ub{i, 1}{j, 1});
                    if ~(u_ub_int.sup < 0)
                        % add constraint and gradient
                        msg = sprintf('ADDED UPPER BOUND INPUT CONSTRAINT ON JOINT %d \n', j);
                        self.vdisp(msg);
                        constraints{end+1, 1} = @(k) slice(self.u_ub{i, 1}{j, 1}, k);
                        grad_u_ub = grad(self.u_ub{i, 1}{j, 1}, self.n_q);
                        grad_constraints{end+1, 1} = @(k) cellfun(@(C) slice(C, k), grad_u_ub);
                    end

                    u_lb_int = interval(self.u_lb{i, 1}{j, 1});
                    if ~(u_lb_int.sup < 0)
                        % add constraint and gradient
                        msg = sprintf('ADDED LOWER BOUND INPUT CONSTRAINT ON JOINT %d \n', j);
                        self.vdisp(msg);
                        constraints{end+1, 1} = @(k) slice(self.u_lb{i, 1}{j, 1}, k);
                        grad_u_lb = grad(self.u_lb{i, 1}{j, 1}, self.n_q);
                        grad_constraints{end+1, 1} = @(k) cellfun(@(C) slice(C, k), grad_u_lb);
                    end
                end
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

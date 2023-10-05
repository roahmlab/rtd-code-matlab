classdef JLSInstance < rtd.planner.reachsets.ReachSetInstance & rtd.util.mixins.NamedClass
    % IRSInstance
    % This is just an individual instance of input reachable set from
    % armour.
    properties
        input_range = [-1.0, 1.0]
        num_parameters = 0
        
        % properties carried over from the original implementation
        q_ub
        q_lb
        dq_ub
        dq_lb
        n_q
        n_t
    end
    methods
        function self = JLSInstance( ...
                    q_ub, q_lb, dq_ub, dq_lb, jrsInstance, verbosity ...
                )
            self.q_ub = q_ub;
            self.q_lb = q_lb;
            self.dq_ub = dq_ub;
            self.dq_lb = dq_lb;
            self.n_q = jrsInstance.n_q;
            self.n_t = jrsInstance.n_t;
            self.num_parameters = jrsInstance.num_parameters;
            
            self.input_range = jrsInstance.input_range;
            self.set_vdisplevel(verbosity);
        end
        
        % Generates an nlconstraint if needed, or will return a NOP
        % function.
        % Returns a function handle for the nlconstraint generated
        % where the function's return type is [c, ceq, gc, gceq]
        function nlconFunction = genNLConstraint(self, worldState)
            constraints = {}; % cell will contain functions of $k$ for evaluating constraints
            grad_constraints = {}; % cell will contain functions of $k$ for evaluating gradients

            % joint limit constraints
            for i = 1:self.n_t
                for j = 1:self.n_q
                    % check if constraint necessary, then add
                    q_ub_int = interval(self.q_ub{i, 1}{j, 1});
                    if ~(q_ub_int.sup < 0)
                        msg = sprintf('ADDED UPPER BOUND JOINT POSITION CONSTRAINT ON JOINT %d AT TIME %d \n', j, i);
                        self.vdisp(msg, 'DEBUG')
                        constraints{end+1, 1} = @(k) slice(self.q_ub{i, 1}{j, 1}, k);
                        grad_q_ub = grad(self.q_ub{i, 1}{j, 1}, self.num_parameters);
                        grad_constraints{end+1, 1} = @(k) cellfun(@(C) slice(C, k), grad_q_ub);
                    end
                    
                    q_lb_int = interval(self.q_lb{i, 1}{j, 1});
                    if ~(q_lb_int.sup < 0)
                        msg = sprintf('ADDED LOWER BOUND JOINT POSITION CONSTRAINT ON JOINT %d AT TIME %d \n', j, i);
                        self.vdisp(msg, 'DEBUG')
                        constraints{end+1, 1} = @(k) slice(self.q_lb{i, 1}{j, 1}, k);
                        grad_q_lb = grad(self.q_lb{i, 1}{j, 1}, self.num_parameters);
                        grad_constraints{end+1, 1} = @(k) cellfun(@(C) slice(C, k), grad_q_lb);
                    end
                    
                    dq_ub_int = interval(self.dq_ub{i, 1}{j, 1});
                    if ~(dq_ub_int.sup < 0)
                        msg = sprintf('ADDED UPPER BOUND JOINT VELOCITY CONSTRAINT ON JOINT %d AT TIME %d \n', j, i);
                        self.vdisp(msg, 'DEBUG')
                        constraints{end+1, 1} = @(k) slice(self.dq_ub{i, 1}{j, 1}, k);
                        grad_dq_ub = grad(self.dq_ub{i, 1}{j, 1}, self.num_parameters);
                        grad_constraints{end+1, 1} = @(k) cellfun(@(C) slice(C, k), grad_dq_ub);
                    end
                    
                    dq_lb_int = interval(self.dq_lb{i, 1}{j, 1});
                    if ~(dq_lb_int.sup < 0)
                        msg = sprintf('ADDED LOWER BOUND JOINT VELOCITY CONSTRAINT ON JOINT %d AT TIME %d \n', j, i);
                        self.vdisp(msg, 'DEBUG')
                        constraints{end+1, 1} = @(k) slice(self.dq_lb{i, 1}{j, 1}, k);
                        grad_dq_lb = grad(self.dq_lb{i, 1}{j, 1}, self.num_parameters);
                        grad_constraints{end+1, 1} = @(k) cellfun(@(C) slice(C, k), grad_dq_lb);
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

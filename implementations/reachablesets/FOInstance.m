classdef FOInstance < ReachableSetInstance
    % FOInstance
    % This is just an individual instance of an original ARMTD FO
    % generation.
    properties
        parameter_range = [-1.0, 1.0]
        output_range = []
        n_k = []
        
        % properties carried over from the original implementation
        robotInfo
        R_w
        p_w
        FO
        jrsInstance
        smooth_obs
        
        obs_frs_combs
    end
    methods
        function self = FOInstance( ...
                    robotInfo, R_w, p_w, FO, jrsInstance, smooth_obs ...
                )
            self.robotInfo = robotInfo;
            self.R_w = R_w;
            self.p_w = p_w;
            self.FO = FO;
            self.jrsInstance = jrsInstance;
            self.smooth_obs = smooth_obs;
            self.n_k = jrsInstance.n_k;
            
            self.parameter_range = jrsInstance.parameter_range;
            % self.output_range = jrsInstance.output_range;
            
            % initialize combinations (for obstacle avoidance constraints)
            self.obs_frs_combs.maxcombs = 200;
            self.obs_frs_combs.combs = generate_combinations_upto(200);
        end
        
        % Handles the obstacle-frs pair or similar to generate the
        % nlconstraint.
        % Returns a function handle for the nlconstraint generated
        % where the function's return type is [c, ceq, gc, gceq]
        function nlconFunction = genNLConstraint(self, worldState)
            
            if ~self.smooth_obs
                obs_constraints = {};
            else
                smooth_obs_constraints = {};
                smooth_obs_constraints_A = {};
                smooth_obs_lambda_index = {};
            end

            % obstacle avoidance constraints
            for i = 1:self.jrsInstance.n_t
                for j = 1:self.robotInfo.params.pz_nominal.num_bodies
                    for o = 1:length(worldState.obstacles) % for each obstacle
                        
                        % first, check if constraint is necessary
                        O_buf = [worldState.obstacles{o}.Z, self.FO{i, 1}{j, 1}.G, self.FO{i, 1}{j, 1}.Grest];
                        [A_obs, b_obs] =  polytope_PH(O_buf, self.obs_frs_combs); % get polytope form
                        if ~(all(A_obs*self.FO{i, 1}{j, 1}.c - b_obs <= 0, 1))
                            continue;
                        end
                        
                        % reduce FO so that polytope_PH has fewer
                        % directions to consider
                        self.FO{i, 1}{j, 1} = reduce(self.FO{i, 1}{j, 1}, 'girard', 3);
                        
                        % now create constraint
                        FO_buf = self.FO{i, 1}{j, 1}.Grest; % will buffer by non-sliceable gens
                        O_buf = [worldState.obstacles{o}.Z, FO_buf]; % describes buffered obstacle zonotope
                        [A_obs, b_obs] = polytope_PH(O_buf, self.obs_frs_combs); % get polytope form

                        % constraint PZ:
                        FO_tmp = polyZonotope_ROAHM(self.FO{i, 1}{j, 1}.c, self.FO{i, 1}{j, 1}.G, [], self.FO{i, 1}{j, 1}.expMat, self.FO{i, 1}{j, 1}.id);
                        obs_constraint_pz = A_obs*FO_tmp - b_obs;

                        % turn into function
                        obs_constraint_pz_slice = @(k) slice(obs_constraint_pz, k);

                        % add gradients
                        grad_obs_constraint_pz = grad(obs_constraint_pz, self.jrsInstance.n_q);
                        grad_obs_constraint_pz_slice = @(k) cellfun(@(C) slice(C, k), grad_obs_constraint_pz, 'UniformOutput', false);
                        
                        % save
                        if ~self.smooth_obs
                            obs_constraints{end+1, 1} = @(k) self.indiv_obs_constraint(obs_constraint_pz_slice, grad_obs_constraint_pz_slice, k);
                        else
                            smooth_obs_constraints_A{end+1, 1} = A_obs;
                            smooth_obs_constraints{end+1, 1} = @(k, lambda) self.indiv_smooth_obs_constraint(obs_constraint_pz_slice, grad_obs_constraint_pz_slice, k, lambda);
                            if isempty(smooth_obs_lambda_index)
                                smooth_obs_lambda_index{end+1, 1} = (1:size(obs_constraint_pz.c, 1))';
                            else
                                smooth_obs_lambda_index{end+1, 1} = smooth_obs_lambda_index{end}(end, 1) + (1:size(obs_constraint_pz.c, 1))';
                            end
                        end
                    end
                end
            end
            % update n_k and parameter_range if smooth
            if self.smooth_obs
                self.n_k = self.jrsInstance.n_k + smooth_obs_lambda_index{end}(end);
                lambda_range = ones(5, 2) .* [0.0, 1.0];
                self.parameter_range = [self.jrsInstance.parameter_range; lambda_range];
            end
            % create the constraint callback
            if self.smooth_obs
                nlconFunction = ...
                    @(k) eval_smooth_constraint( ...
                        k(1:self.jrsInstance.n_k),...
                        k(self.jrsInstance.n_k+1:end),...
                        smooth_obs_constraints_A,...
                        smooth_obs_constraints,...
                        smooth_obs_lambda_index);
            else
                nlconFunction = @(k) eval_constraint(k, length(obs_constraints), obs_constraints);
            end
        end
        
        function [h_obs, grad_h_obs] = indiv_obs_constraint(self, c, grad_c, k)
            % made a separate function to handle the obstacle constraints,
            % because the gradient requires knowing the index causing
            % the max of the constraints
            [h_obs, max_idx] = max(c(k));
            h_obs = -h_obs;
            grad_eval = grad_c(k);
            grad_h_obs = zeros(self.jrsInstance.n_k, 1);
            for i = 1:self.jrsInstance.n_k
                grad_h_obs(i, 1) = -grad_eval{i}(max_idx, :);
            end
        end

        function [h_obs, grad_h_obs] = indiv_smooth_obs_constraint(self, c, grad_c, k, lambda)
            % for evaluating smooth obs constraints, where lambda is introduced
            % as extra decision variables to avoid taking a max.

            % evaluate constraint: (A*FO - b)'*lambda
            h_obs = -c(k)'*lambda;

            % evaluate gradient w.r.t. k... 
            % grad_c(k) gives n_k x 1 cell, each containing
            % an N x 1 vector, where N is the number of rows of A.
            % take the dot product of each cell with lambda
            grad_eval = grad_c(k);
            grad_h_obs = zeros(self.jrsInstance.n_k, 1);
            for i = 1:self.jrsInstance.n_k
                grad_h_obs(i, 1) = -grad_eval{i}'*lambda;
            end

            % evaluate gradient w.r.t. lambda...
            % this is just (A*FO-b)
            grad_h_obs = [grad_h_obs; -c(k)];
        end
        
    end
end

function [h, heq, grad_h, grad_heq] = eval_constraint(k, n_c, obs_constraints)
    h = zeros(n_c, 1);
    grad_h = zeros(length(k), n_c);

    for i = 1:n_c
        [h_i, grad_h_i] = obs_constraints{i}(k);
        h(i) = h_i;
        grad_h(:, i) = grad_h_i;
    end

    grad_heq = [];
    heq = [];
end

function [h, heq, grad_h, grad_heq] = eval_smooth_constraint(k, lambda, ...
    smooth_obs_constraints_A, smooth_obs_constraints, smooth_obs_lambda_index)

    n_obs_c = length(smooth_obs_constraints);
    n_k = length(k);
    n_lambda = length(lambda);

    % max_lambda_index = P.smooth_obs_lambda_index{end}(end);

    % number of constraints: 
    % - 1 obstacle avoidance per obstacle
    % - 1 sum lambdas for obstacle = 1
    % - n_lambda lambda \in {0, 1}

    h = zeros(2*n_obs_c, 1);
    grad_h = zeros(n_k + n_lambda, 2*n_obs_c);

    for i = 1:n_obs_c
        lambda_idx = smooth_obs_lambda_index{i};
        lambda_i = lambda(lambda_idx, 1);% pull out correct lambdas!!
        [h_i, grad_h_i] = smooth_obs_constraints{i}(k, lambda_i);

        % obs avoidance constraints:
        h(i, 1) = h_i;
        grad_h(1:n_k, i) = grad_h_i(1:n_k, 1);
        grad_h(n_k + lambda_idx, i) = grad_h_i(n_k+1:end, 1);

        % sum lambdas for this obstacle constraint >= 1
        % sum lambdas for this obstacle constraint <= 1?
%                 h(n_obs_c + i, 1) = 1 - sum(lambda_i, 1); 
%                 grad_h(n_k + lambda_idx, n_obs_c + i) = -ones(length(lambda_i), 1);

        % from Borrelli paper
        h(n_obs_c + i, 1) = norm(smooth_obs_constraints_A{i}'*lambda_i, 2) - 1;
        % implement gradient here!!!
        A_bar = smooth_obs_constraints_A{i}*smooth_obs_constraints_A{i}';
        grad_h(n_k + lambda_idx, n_obs_c + i) = 0.5*(lambda_i'*A_bar*lambda_i)^(-0.5)*2*A_bar*lambda_i;

    end

    % lambda \in {0, 1} for each lambda
%             heq = lambda.*(lambda - 1);
%             grad_heq = zeros(n_k + n_lambda, n_lambda);
%             grad_heq(n_k+1:end, :) = diag(2*lambda - 1);

     heq = [];
     grad_heq = [];
end
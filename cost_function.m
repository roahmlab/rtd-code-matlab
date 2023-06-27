classdef cost_function < rtd.planner.trajopt.Objective
    properties
        cost
        grad_cost
    end
    methods
        function objectiveCallback = genObjective(obj)
            objectiveCallback = @(params) obj.evaluateObjective(params);
        end

        function cost_ = evaluateObjective(obj, params)
            [zono_center_xyh, zono_center_u0v0r0p, zono_u0v0r0p_slice_gens_xyh, ...
                zono_u0v0r0p_slice_gens_slice, u0v0r0, param_val, target_pose, ...
                weight_h, weight_xy, h_epsilon, xy_epsilon] = deal(params{:});

            % Validate input sizes
            assert(all(size(zono_center_xyh) == [3 1]));
            assert(all(size(zono_center_u0v0r0p) == [4 1]));
            assert(all(size(zono_u0v0r0p_slice_gens_xyh) == [3 4]));
            assert(all(size(zono_u0v0r0p_slice_gens_slice) == [4 1]));
            assert(all(size(u0v0r0) == [3 1]));
            assert(all(size(param_val) == [1 1]));
            assert(all(size(target_pose) == [3 1]));
            assert(all(size(weight_h) == [1 1]));
            assert(all(size(weight_xy) == [1 1]));

            u0v0r0p_lambda_vals = ([u0v0r0; param_val] - zono_center_u0v0r0p) ./ zono_u0v0r0p_slice_gens_slice;

            if isnumeric(u0v0r0p_lambda_vals)
                assert(all(abs(u0v0r0p_lambda_vals) <= 1.0 + 1.0e-4));
            end

            u0v0r0p_lambda_vals_transpose = u0v0r0p_lambda_vals';
            u0v0r0p_delta_xyh = sum(u0v0r0p_lambda_vals_transpose .* zono_u0v0r0p_slice_gens_xyh, 2);

            new_pose = zono_center_xyh + u0v0r0p_delta_xyh;

            error = target_pose - new_pose;

            cost = weight_h .* sqrt(error(3).^2 + h_epsilon) + weight_xy .* sqrt(sum(error(1:2).^2) + xy_epsilon);

            % Compute the gradient if requested
            if nargout > 1
                % % Compute the derivatives, if requested
                % dcost = diff(cost, param_val);
                % 
                % % Compute the gradient
                % grad_cost = double(subs(dcost, param_val, params{6}));

                syms param_val_sym;
                error_sym = sym('error', [3 1], 'real');
                cost_sym = weight_h .* sqrt(error_sym(3).^2 + h_epsilon) + weight_xy .* sqrt(sum(error_sym(1:2).^2) + xy_epsilon);
                dcost_sym = diff(cost_sym, param_val_sym);
                d2cost_sym = diff(dcost_sym, param_val_sym);

                % Evaluate the derivatives at the given parameter value
                grad_cost = double(subs(dcost_sym, param_val_sym, param_val));
                d2cost = double(subs(d2cost_sym, param_val_sym, param_val));
            else
                grad_cost = 0;
                d2cost =0;
            end
            cost_.cost = cost;
            cost_.grad_cost = grad_cost;
            
        end
    end
end

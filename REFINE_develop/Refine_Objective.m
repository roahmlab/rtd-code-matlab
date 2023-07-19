

classdef Refine_Objective < rtd.planner.trajopt.Objective
    properties
        desired_idx
        vehrs
        cost
        grad_cost
        t_cost
        params
        weight_h = 3;
        weight_xy = 10;
        h_epsilon = 1.0e-6;
        xy_epsilon = 1.0e-6;
    end
    methods
        function self = Refine_Objective(trajOptProps, vehrs, desired_idx)
            arguments
                trajOptProps rtd.planner.trajopt.TrajOptProps
                vehrs
                desired_idx
            end
            self.t_cost = trajOptProps.timeForCost;
            self.vehrs = vehrs;
            self.desired_idx = desired_idx;
        end
        
        function objectiveCallback = genObjective(self)
            for i = 1:length(self.vehrs)
                zono = self.vehrs{i}{self.desired_idx};
                z_matrix = zono.z_matrix;
                zono_center_xyh = z_matrix(1:3, 1);
                zono_center_u0v0r0p = z_matrix(4:7, 1);
                
                u0_non_zero_idx = find(z_matrix(7, 2:end) ~= 0, 1);
                v0_non_zero_idx = find(z_matrix(8, 2:end) ~= 0, 1);
                r0_non_zero_idx = find(z_matrix(9, 2:end) ~= 0, 1);
                p_non_zero_idx = find(z_matrix(10, 2:end) ~= 0, 1);

                slice_point = [u0_non_zero_idx; v0_non_zero_idx; r0_non_zero_idx];
                slice_dim = [1; 2; 3];
                zono_u0v0r0p_slice_gens_xyh = zonotope_slice(zono, slice_dim, slice_point);
                zono_u0v0r0p_slice_gens_slice = [z_matrix(7, u0_non_zero_idx); z_matrix(8, u0_non_zero_idx); z_matrix(9, u0_non_zero_idx); z_matrix(10, p_non_zero_idx)];
                param_val = 11.3;
                u0v0r0 = z_matrix(4:6, 1);
                target_pose = [3; 4; 2]; % Replace with appropriate values for target pose
                self.params = {zono_center_xyh, zono_center_u0v0r0p, zono_u0v0r0p_slice_gens_xyh, ...
                    zono_u0v0r0p_slice_gens_slice, u0v0r0, param_val, target_pose, ...
                    self.weight_h, self.weight_xy, self.h_epsilon, self.xy_epsilon};
                objectiveCallback = @(params) self.evaluateObjective(params);
            end
        end

        function [cost_, grad_cost_] = evaluateObjective(self, params)
            [zono_center_xyh, zono_center_u0v0r0p, zono_u0v0r0p_slice_gens_xyh, ...
                zono_u0v0r0p_slice_gens_slice, u0v0r0, param_val, target_pose, ...
                self.weight_h, self.weight_xy, self.h_epsilon, self.xy_epsilon] = deal(params{:});

            u0v0r0p_lambda_vals = ([u0v0r0; param_val] - zono_center_u0v0r0p) ./ zono_u0v0r0p_slice_gens_slice;

            if isnumeric(u0v0r0p_lambda_vals)
                assert(all(abs(u0v0r0p_lambda_vals) <= 1.0 + 1.0e-4));
            end

            u0v0r0p_lambda_vals_transpose = u0v0r0p_lambda_vals';
            u0v0r0p_delta_xyh = sum(u0v0r0p_lambda_vals_transpose .* zono_u0v0r0p_slice_gens_xyh, 2);

            new_pose = zono_center_xyh + u0v0r0p_delta_xyh;

            error = target_pose - new_pose;

            self.cost = self.weight_h .* sqrt(error(3).^2 + self.h_epsilon) + self.weight_xy .* sqrt(sum(error(1:2).^2) + self.xy_epsilon);
            cost_ = self.cost;

            syms param_val_sym;
            error_sym = sym('error', [3 1], 'real');
            cost_sym = self.weight_h .* sqrt(error_sym(3).^2 + self.h_epsilon) + self.weight_xy .* sqrt(sum(error_sym(1:2).^2) + self.xy_epsilon);
            dcost_sym = diff(cost_sym, param_val_sym);

            self.grad_cost = double(subs(dcost_sym, param_val_sym, param_val));
            grad_cost_ = self.grad_cost;
        end
    end  
end


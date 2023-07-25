

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
        
        function objectiveCallback = genObjective(self,robotState, ...
                    waypoint, rsInstances)

            for i = 1:length(self.vehrs)
               
                zono = self.vehrs{i};
                
                z_matrix = zono.center + zono.generators;
                zono_center_xyh = z_matrix(1:3, 1);
                zono_center_u0v0r0p = z_matrix(4:7, 1);
%                 disp(z_matrix)
                % call zonotope_slice
                %not sure if i need to passin the center values at this
                %dimension or generator values
%                 new_zono{i} = zonotope_slice(zono{i},[7;8;9;11],[z_matrix(7,1);z_matrix(8,1);z_matrix(9,1);z_matrix(11,1)]);
%                 disp('new_zono')
%                 disp(new_zono{1}.Z)
                % fprintf('New_zono = %d \n',new_zono);
                
                %index of non-zero u0,v0,r0,p
                u0_non_zero_idx = find(z_matrix(7, 2:end) ,1,'first');
                v0_non_zero_idx = find(z_matrix(8, 2:end) ,1,'first');
                r0_non_zero_idx = find(z_matrix(9, 2:end) ,1,'first');
                p_non_zero_idx  = find(z_matrix(11, 2:end) ,1,'first');

                %values
                u0_xyh = z_matrix(1:3,u0_non_zero_idx+1);
                v0_xyh = z_matrix(1:3,v0_non_zero_idx+1);
                r0_xyh = z_matrix(1:3,r0_non_zero_idx+1);
                p_xyh  = z_matrix(1:3,p_non_zero_idx+1);

                zono_u0v0r0p_slice_gens_xyh = [u0_xyh.'; v0_xyh.'; r0_xyh.'; p_xyh.']';


                % disp(size(zono_u0v0r0p_slice_gens_xyh))
                zono_u0v0r0p_slice_gens_slice = [z_matrix(7, u0_non_zero_idx+1); z_matrix(8, v0_non_zero_idx+1); z_matrix(9, r0_non_zero_idx+1); z_matrix(11, p_non_zero_idx+1)];
                % disp(size(zono_u0v0r0p_slice_gens_slice))
                param_val = 13;
                u0v0r0 = z_matrix(4:6, 1);
                target_pose = [3; 4; 2]; % Replace with appropriate values for target pose
                self.params = {zono_center_xyh, zono_center_u0v0r0p, zono_u0v0r0p_slice_gens_xyh, ...
                    zono_u0v0r0p_slice_gens_slice, u0v0r0, param_val, target_pose, ...
                    self.weight_h, self.weight_xy, self.h_epsilon, self.xy_epsilon};
                objectiveCallback = @(params) self.evaluateObjective(self.params);
            end
        end

        function cost_ = evaluateObjective(self, params)
            [zono_center_xyh, zono_center_u0v0r0p, zono_u0v0r0p_slice_gens_xyh, ...
                zono_u0v0r0p_slice_gens_slice, u0v0r0, param_val, target_pose, ...
                self.weight_h, self.weight_xy, self.h_epsilon, self.xy_epsilon] = deal(params{:});

           %%from here
            u0v0r0p_lambda_vals = ([u0v0r0; param_val] - zono_center_u0v0r0p) ./ zono_u0v0r0p_slice_gens_slice;

            if isnumeric(u0v0r0p_lambda_vals)
                assert(all(abs(u0v0r0p_lambda_vals) <= 1.0 + 1.0e-4), 'u0v0r0p_lambda_vals is out of range');
            end

            u0v0r0p_lambda_vals_transpose = u0v0r0p_lambda_vals';
            u0v0r0p_delta_xyh = sum(u0v0r0p_lambda_vals_transpose .* zono_u0v0r0p_slice_gens_xyh, 2);

            new_pose = zono_center_xyh + u0v0r0p_delta_xyh;

            error = target_pose - new_pose;

            self.cost = self.weight_h .* sqrt(error(3).^2 + self.h_epsilon) + self.weight_xy .* sqrt(sum(error(1:2).^2) + self.xy_epsilon);
            cost_ = self.cost;

        end
    end  
end




classdef Refine_Objective < rtd.planner.trajopt.Objective
    properties
        desired_idx
        vehrs
        cost
        t_cost
        params
    end

    properties
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
        
        function objectiveCallback = genObjective(self,robotState,waypoint, rsInstances)

                zono = self.vehrs{self.desired_idx};%taking the zonos of the desired idx only

                z_matrix = zono.Z;
                zono_center_xyh = z_matrix(1:3, 1);%we can take the center only from the zonotope_slice directly
                zono_center_u0v0r0p = z_matrix([7,8,9,11], 1);%after slicing 

                %zonotope_slicing
                [zono_zero_u0v0r0_gen,slice_idx1] = zonotope_slice(zono,[7,8,9,11],[z_matrix(7, 1); z_matrix(8, 1); z_matrix(9, 1); z_matrix(11, 1)]);%zonotope without non-zero generators of u0,v0,r0,p
        
                %zono difference for u0,v0,r0,p generators
                zono_diff = zeros(20,length(slice_idx1));
                for i = 1:length(slice_idx1)
                    zono_diff(:,i) = z_matrix(:,slice_idx1(i,1)+1);
                end


                %values from the zono_diff
                u0_xyh = zono_diff(1:3,1);
                v0_xyh = zono_diff(1:3,2);
                r0_xyh = zono_diff(1:3,3);
                p_xyh  = zono_diff(1:3,4);

                zono_u0v0r0p_slice_gens_xyh = [u0_xyh.'; v0_xyh.'; r0_xyh.'; p_xyh.']';%after zonotope_slice
                zono_u0v0r0p_slice_gens_slice = [zono_diff(7, 1); zono_diff(8, 2); zono_diff(9, 3); zono_diff(11,4)];%after zonotope_slice

                param_val = 12.28;%constant for the first iteration and then updated in the evaluateConstraints

                u0v0r0 = z_matrix(7:9,1);

                %my target pose is my waypoint. 
                target_pose = waypoint(1,:).'; %take from the next waypoint. print and check


                objectiveCallback = @(params) self.evaluateObjective(params, zono_center_xyh, zono_center_u0v0r0p, zono_u0v0r0p_slice_gens_xyh, ...
                zono_u0v0r0p_slice_gens_slice, u0v0r0, param_val, target_pose);

        end

        function cost_ = evaluateObjective(self, params,zono_center_xyh, zono_center_u0v0r0p, zono_u0v0r0p_slice_gens_xyh, ...
                zono_u0v0r0p_slice_gens_slice, u0v0r0, param_val, target_pose)

           %%from here
           
%            disp([u0v0r0; param_val])
%            disp(zono_center_u0v0r0p)
%            disp(zono_u0v0r0p_slice_gens_slice)
            u0v0r0p_lambda_vals = ([u0v0r0; param_val] - zono_center_u0v0r0p) ./ zono_u0v0r0p_slice_gens_slice;


            param_val = params(end);
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


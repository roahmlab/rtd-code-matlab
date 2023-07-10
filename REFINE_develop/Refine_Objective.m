% 
% classdef Refine_Objective < rtd.planner.trajopt.Objective
%     properties
%         desired_idx
%         vehrs
%         cost
%         grad_cost
%         t_cost
%         params
%         weight_h = 3;
%         weight_xy = 10;
%         h_epsilon = 1.0e-6;
%         xy_epsilon = 1.0e-6;
%     end
%     methods
%         function self = Refine_Objective(trajOptProps,vehrs,desired_idx)
%             arguments
%                 trajOptProps rtd.planner.trajopt.TrajOptProps
%                 vehrs
%                 desired_idx
%                 % trajectoryFactory TrajectoryFactory
%             end
%             % self.trajectoryFactory = trajectoryFactory;
%             self.t_cost = trajOptProps.timeForCost;
%             self.vehrs = vehrs;
%             self.desired_idx = desired_idx;
%             % disp(self.t_cost)
%         end
% 
%         %why do we need genObjective, check other files/cost calculation
%         %we need genObjective in RtdTrajOpt
%         %here vehrs is not being passed.
%         %we do not have vehrs even in constructor
%         %how do i calculate zono slice?
% 
%         function objectiveCallback = genObjective(self,RobotState,waypoint,reachableSets)%get vehrs here
%             % rs= reachableSets.z_matrix;
%             % for i = length(reachableSets.z_matrix)
%                 %ASK IF t_cost and desired_idx is the same?
%                 % for j = length(reachableSets.z_matrix{i})
%                 %     % x = reachableSets.z_matrix{i}{j}
%                 %     % disp(reachableSets.z_matrix{i}{i})
%                 %     % disp(reachableSets{i})
%                 % 
%                 %     for zono=reachableSets.z_matrix{i}{j}
%                 %         if zono ~= self.t_cost
%                 %             continue
%                 %         else
%                 %             % From slicing reachableSets at desired_idx / t_cost
%                 %             disp('found t_cost')
%                 %             disp(zono)
%                 %             zono_center_xyh = zono(1:3)
%                 %             zono_center_u0v0r0p = zono(4:7);
%                 %             zono_u0v0r0p_slice_gens_xyh = [1 2 3 4; 5 6 7 8; 9 10 11 12];
%                 %             zono_u0v0r0p_slice_gens_slice = [1; 0.1; 0.1; 2];
%                 %             break
%                 %         end
%                 % 
%                 %     end
% 
% 
% 
%                     for i = 1:length(self.vehrs)
%                         zono = self.vehrs{i}{self.desired_idx};
%                         z_matrix = zono.z_matrix;%check the order of this/calling
%                         %extract the values as we need them
%                         zono_center_xyh = z_matrix(1:3,1);
%                         zono_center_u0v0r0p = z_matrix(4:7,1);
% 
%                         %the first non zero generator value for the below
%                         u0_non_zero_idx = find(z_matrix(7, 2:end) ~= 0, 1);
%                         v0_non_zero_idx = find(z_matrix(8, 2:end) ~= 0, 1);
%                         r0_non_zero_idx = find(z_matrix(9, 2:end) ~= 0, 1);
%                         p_non_zero_idx = find(z_matrix(10, 2:end) ~= 0, 1);
% 
%                         %store all the index together
%                         slice_point = [u0_non_zero_idx;v0_non_zero_idx;r0_non_zero_idx];
%                         slice_dim = [1;2;3];
%                         zono_u0v0r0p_slice_gens_xyh = zonotope_slice(zono,slice_dim,slice_point);
%                         zono_u0v0r0p_slice_gens_slice = [z_matrix[7,u0_non_zero_idx];z_matrix[8,u0_non_zero_idx];z_matrix[9,u0_non_zero_idx];z_matrix[10,p_non_zero_idx]];
%                         param_val = 11.3;
%                         %ASK BELOW
%                         u0v0r0 = z_matrix(4:6,1);
%                         self.params = {zono_center_xyh, zono_center_u0v0r0p, zono_u0v0r0p_slice_gens_xyh, ...
%                         zono_u0v0r0p_slice_gens_slice, u0v0r0, target_pose, ...
%                         self.weight_h, self.weight_xy, self.h_epsilon, self.xy_epsilon};
%                         objectiveCallback = @(params) self.evaluateObjective({zono_center_xyh, zono_center_u0v0r0p, zono_u0v0r0p_slice_gens_xyh, ...
%                                 zono_u0v0r0p_slice_gens_slice, u0v0r0, param_val, target_pose, self.weight_h, self.weight_xy, self.h_epsilon, self.xy_epsilon});
% 
%                     end
% 
% 
%          end
% 
% 
%         function [cost_, grad_cost_] = evaluateObjective(obj,params)
% 
%             [zono_center_xyh, zono_center_u0v0r0p, zono_u0v0r0p_slice_gens_xyh, ...
%                 zono_u0v0r0p_slice_gens_slice, u0v0r0, param_val, target_pose, ...
%                 obj.weight_h, obj.weight_xy, obj.h_epsilon, obj.xy_epsilon] = deal(params{:});
% 
%             % Validate input sizes
%             assert(all(size(zono_center_xyh) == [3 1]));
%             assert(all(size(zono_center_u0v0r0p) == [4 1]));
%             assert(all(size(zono_u0v0r0p_slice_gens_xyh) == [3 4]));
%             assert(all(size(zono_u0v0r0p_slice_gens_slice) == [4 1]));
%             assert(all(size(u0v0r0) == [3 1]));
%             assert(all(size(param_val) == [1 1]));
%             assert(all(size(target_pose) == [3 1]));
%             assert(all(size(obj.weight_h) == [1 1]));
%             assert(all(size(obj.weight_xy) == [1 1]));
% 
%             u0v0r0p_lambda_vals = ([u0v0r0; param_val] - zono_center_u0v0r0p) ./ zono_u0v0r0p_slice_gens_slice;
% 
%             if isnumeric(u0v0r0p_lambda_vals)
%                 assert(all(abs(u0v0r0p_lambda_vals) <= 1.0 + 1.0e-4));
%             end
% 
%             u0v0r0p_lambda_vals_transpose = u0v0r0p_lambda_vals';
%             u0v0r0p_delta_xyh = sum(u0v0r0p_lambda_vals_transpose .* zono_u0v0r0p_slice_gens_xyh, 2);
% 
%             new_pose = zono_center_xyh + u0v0r0p_delta_xyh;
% 
%             error = target_pose - new_pose;
% 
%             obj.cost = obj.weight_h .* sqrt(error(3).^2 + obj.h_epsilon) + obj.weight_xy .* sqrt(sum(error(1:2).^2) + obj.xy_epsilon);
%             cost_ = obj.cost;
%             % Compute the gradient if requested
%             % if nargout > 1
%                 % % Compute the derivatives, if requested
%                 % dcost = diff(cost, param_val);
%                 % 
%                 % % Compute the gradient
%                 % grad_cost = double(subs(dcost, param_val, params{6}));
% 
%                 syms param_val_sym;
%                 error_sym = sym('error', [3 1], 'real');
%                 cost_sym = obj.weight_h .* sqrt(error_sym(3).^2 + obj.h_epsilon) + obj.weight_xy .* sqrt(sum(error_sym(1:2).^2) + obj.xy_epsilon);
%                 dcost_sym = diff(cost_sym, param_val_sym);
%                 d2cost_sym = diff(dcost_sym, param_val_sym);
% 
%                 % Evaluate the derivatives at the given parameter value
%                 obj.grad_cost = double(subs(dcost_sym, param_val_sym, param_val));
%                 d2cost = double(subs(d2cost_sym, param_val_sym, param_val));
%             % else
%             %     obj.grad_cost = 0;
%             %     d2cost =0;
%             % end
%             grad_cost_ = obj.grad_cost;
% 
%             disp('end of evaluate');
%         end
% 
%     end  
% end
% 
% 

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


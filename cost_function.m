
clear all;
clc;

%% Testing of cost function

% Dummy variables
vehrs = struct();
vehrs.slc_infos_{1} = struct('Slice', @(u,v,r) struct('x_sliced_sum_', 1, 'y_sliced_sum_', 2, 'h_sliced_sum_', 3, 'slc_x_', 4, 'slc_y_', 5, 'slc_h_', 6, 'slc_val_', 7));
vehrs.slc_infos_{2} = struct('Slice', @(u,v,r) struct('x_sliced_sum_', 8, 'y_sliced_sum_', 9, 'h_sliced_sum_', 10, 'slc_x_', 11, 'slc_y_', 12, 'slc_h_', 13, 'slc_val_', 14));
vehrs.xy_centers_{1} = 10;
vehrs.xy_centers_{2} = 20;
vehrs.slc_infos_{3} = struct('Slice', @(u,v,r) struct('x_sliced_sum_', 1, 'y_sliced_sum_', 2, 'h_sliced_sum_', 3, 'slc_x_', 4, 'slc_y_', 5, 'slc_h_', 6, 'slc_val_', 7));
vehrs.slc_infos_{4} = struct('Slice', @(u,v,r) struct('x_sliced_sum_', 8, 'y_sliced_sum_', 9, 'h_sliced_sum_', 10, 'slc_x_', 11, 'slc_y_', 12, 'slc_h_', 13, 'slc_val_', 14));
vehrs.xy_centers_{3} = 10;
vehrs.xy_centers_{4} = 20;
vehrs.slc_infos_{5} = struct('Slice', @(u,v,r) struct('x_sliced_sum_', 1, 'y_sliced_sum_', 2, 'h_sliced_sum_', 3, 'slc_x_', 4, 'slc_y_', 5, 'slc_h_', 6, 'slc_val_', 7));
vehrs.slc_infos_{6} = struct('Slice', @(u,v,r) struct('x_sliced_sum_', 8, 'y_sliced_sum_', 9, 'h_sliced_sum_', 10, 'slc_x_', 11, 'slc_y_', 12, 'slc_h_', 13, 'slc_val_', 14));
vehrs.xy_centers_{5} = 10;
vehrs.xy_centers_{6} = 20;

desired_idx = 2;
state_u = 0;
state_v = 0;
state_r = 0;
x_des = 0;
y_des = 0;
h_des = 0;
k = 0;
k_min = 0;
k_max = 1;
eval_opt = true;

%cost function
gen_cost = GenCostFcn(vehrs, desired_idx, state_u, state_v, state_r, x_des, y_des, h_des);
% Call ComputeDeltaY
delta_y = ComputeDeltaY(gen_cost, k, k_min, k_max);

% Call ComputeDeltaH
delta_h = ComputeDeltaH(gen_cost, k, k_min, k_max);

% Call ComputeCosts
costs = ComputeCosts(gen_cost, k, k_min, k_max, eval_opt);

%% Generate cost function

function generate = GenCostFcn(vehrs,desired_idx,state_u,state_v,state_r,x_des,y_des,h_des)
    generate = struct();
    slc = vehrs.slc_infos_{desired_idx}.Slice(state_u,state_v,state_r);
    disp(['[LAN] Desired Idx: ', num2str(desired_idx)]);
    num_centers = numel(vehrs.xy_centers_);


    if desired_idx >= 1 && desired_idx <= num_centers/2
        c_x = vehrs.xy_centers_{desired_idx *2 +0} +slc.x_sliced_sum_;%assuming slc has x_sliced_sum_
        c_y = vehrs.xy_centers_{desired_idx *2 +1} +slc.y_sliced_sum_;
        c_h = vehrs.xy_centers_{desired_idx} +slc.h_sliced_sum_;
        generate.c_x_ = c_x;
        generate.c_y_ = c_y;
        generate.c_h_ = ToAngle(c_h);
        % generate.c_k_ = slc.center_slc_val_;
        generate.c_k_ = 1;
        generate.g_x_ = slc.slc_x_;
        generate.g_y_ = slc.slc_y_;
        generate.g_h_ = slc.slc_h_;
        generate.g_k_ = slc.slc_val_;
        generate.x_des_ = x_des;
        generate.y_des_ = y_des;
        generate.h_des_ = h_des;
    else
        print('Desired index out of range')
    end    

end
%% Compute Delta Y function

function y = ComputeDeltaY(gen_cost,k,k_min,k_max)
    lambda = (k - gen_cost.c_k_)/gen_cost.g_k_;
    disp(['[LAN] [K: ', num2str(k), '] [C_K: ', num2str(gen_cost.c_k_), '] [G_K: ', num2str(gen_cost.g_k_), '] [Lambda: ', num2str(lambda), '] [C_Y: ', num2str(gen_cost.c_y_), '] [G_Y: ', num2str(gen_cost.g_y_), ']']);
    y = gen_cost.c_y_ + (lambda * gen_cost.g_y_);
end

%% Compute Delta H function

function h = ComputeDeltaH(gen_cost,k,k_min,k_max)
    lambda = (k - gen_cost.c_k_)/gen_cost.g_k_;
    h = gen_cost.c_h_ + (lambda * gen_cost.g_h_);
end
%% To Angle function

function angle_ = ToAngle(theta)
    if(~(isfinite(theta)))
        angle_ = 0;
    else
        angle_ = angle(cos(theta) + 1i * sin(theta));
    end

end

%% Compute Cost function

function ret_val = ComputeCosts(gen_cost,k,k_min,k_max,eval_opt)
    % ret_val = struct('cost',[],'derivs',[]);
    ret_val = struct();
    c_x = gen_cost.c_x_;
    c_y = gen_cost.c_y_;
    c_h = gen_cost.c_h_;
    c_k = gen_cost.c_k_;
    g_x = gen_cost.g_x_;
    g_y = gen_cost.g_y_;
    g_h = gen_cost.g_h_;
    g_k = gen_cost.g_k_;
    gx2 = g_x^2;
    gy2 = g_y^2;
    gh2 = g_h^2;
    gk2 = g_k^2;
    x_des = gen_cost.x_des_;
    y_des = gen_cost.y_des_;
    h_des = gen_cost.h_des_;

    eval_opt = false;

    %if simulation
    weight_xy = 3;
    weight_h = 10;

    ret_val.cost_k = weight_h * sqrt((h_des - c_h + (g_h * (c_k - k))/g_k).^2) + weight_xy * sqrt((x_des - c_x + (g_x * (c_k - k))/g_k).^2) + (y_des - c_y + (g_y * (c_k - k))/g_k).^2;
    
    ret_val.jac_k_ = -(weight_xy * ((2.0 * g_x * (x_des - c_x + (g_x * (c_k - k)) / g_k)) / g_k + (2.0 * g_y * (y_des - c_y + (g_y * (c_k - k)) / g_k)) / g_k)) / (2.0 * sqrt((x_des - c_x + (g_x * (c_k - k)) / g_k).^2 + (y_des - c_y + (g_y * (c_k - k)) / g_k).^2)) - (g_h * weight_h * (h_des - c_h + (g_h * (c_k - k)) / g_k)) / (g_k * sqrt((h_des - c_h + (g_h * (c_k - k)) / g_k).^2));

    ret_val.hess_k_ = (weight_xy * ((2.0 * gx2) / gk2 + (2.0 * gy2) / gk2)) / (2.0 * sqrt((x_des - c_x + (g_x * (c_k - k)) / g_k).^2 + (y_des - c_y + (g_y * (c_k - k)) / g_k).^2)) - (weight_xy * ((2.0 * g_x * (x_des - c_x + (g_x * (c_k - k)) / g_k)) / g_k + (2.0 * g_y * (y_des - c_y + (g_y * (c_k - k)) / g_k)) / g_k).^2) / (4 * ((x_des - c_x + (g_x * (c_k - k)) / g_k).^2 + (y_des - c_y + (g_y * (c_k - k)) / g_k).^2).^(3/2)) + (gh2 * weight_h) / (gk2 * sqrt((h_des - c_h + (g_h * (c_k - k)) / g_k).^2)) - (gh2 * weight_h * (h_des - c_h + (g_h * (c_k - k)) / g_k).^2) / (gk2 * ((h_des - c_h + (g_h * (c_k - k)) / g_k).^2).^(3/2));
    
    % We can have discontinuous derivatives. For this, we set the jacobian
    % to zero at these points.
    handle_discontinuity = @(d) handleDiscontinuity(d);

    ret_val.jac_k_ = handleDiscontinuity(ret_val.jac_k_);
    ret_val.hess_k_ = handleDiscontinuity(ret_val.hess_k_);
    
    function result = handleDiscontinuity(d)
        if ~isfinite(d)
            result = 0.0;
        else
            result = d;
        end
    end

    if(eval_opt == true)
        jac_0 = (c_k * gh2 + c_k * gx2 + c_k * gy2 - c_h * g_h * g_k - c_x * g_k * g_x - c_y * g_k * g_y + g_h * g_k * h_des + g_k * g_x * x_des + g_k * g_y * y_des) / (gh2 + gx2 + gy2);
        optimal_evals = ComputeCosts(jac_0, k_min, k_max, false);
        optimal_cost = optimal_evals.cost_k_;
        optimal_jac = optimal_evals.jac_k_;
        optimal_hess = optimal_evals.hess_k_;
        delta_optimal_cost = ComputeCosts(jac_0 + 0.05, k_min, k_max, false).cost_k_;
        concave_up = optimal_cost < delta_optimal_cost;
        dist_to_min = abs(jac_0 - k_min);
        dist_to_max = abs(jac_0 - k_max);
        jac_0_in_thresh = InClosedInterval(jac_0, k_min, k_max);
        if (concave_up == true)
            if(jac_0_in_thresh == true)
                ret_val.optimal_k_in_rng_ = jac0;
            else
                ret_val.optimal_k_in_rng_ = (dist_to_max > dist_to_min) * k_max + (dist_to_max <= dist_to_min) * k_min;
            end
        else
            ret_val.optimal_k_in_rng_ = (dist_to_max > dist_to_min) * k_min + (dist_to_max <= dist_to_min) * k_max;
        end
        optimal_cost_in_rng = ComputeCosts(ret_val.optimal_k_in_rng_, k_min, k_max, false).cost_k_;
    end
    
end

%% In closed Interval function

function result = InClosedInterval(x, a, b)
    result = (x >= a) && (x <= b);
end

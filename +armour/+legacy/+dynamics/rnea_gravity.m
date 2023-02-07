function g = rnea_gravity(q, robot_params)
import armour.legacy.dynamics.*
    gravity_flag = 1;
    n = length(q);
    g = rnea(q, zeros(1,n), zeros(1,n), zeros(1,n), gravity_flag, robot_params);
end
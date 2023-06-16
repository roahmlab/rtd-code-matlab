

classdef FRS_Instance_speed_change
    properties
        
        Vehrs 
        constraints
    end

    methods
        function self = test2_FRS_Instance_speed_change(Vehrs)
            
            self.Vehrs = Vehrs;
            self.constraints = [];
            Agent = agent();%dummy values for testing only
            agent_info = Agent.get_agent_info();
            W = dynamic_car_world('start_line', 5, 'bounds', [-5 5 0 12], 'N_obstacles', 2);%passing dummy variables
            world_info = W.get_world_info(agent_info);
            self.constraints = genNLConstraint(self,world_info);
        end
        
        
        function constraints = genNLConstraint(self, worldState)
            vehrs = self.Vehrs;
            num_zonotopes = numel(vehrs);
            constraints = cell(1, num_zonotopes);
            
            for each_zono = 1:num_zonotopes
                zono = vehrs{each_zono};
                obs_info = get_obs_mex(self,worldState.dyn_obstacles,worldState.bounds);
                constraints{each_zono} = self.GenerateConstraints(zono,obs_info);
            end
        end
        
        

    end

    methods (Access = private)

        %generate the obs_info
        function obj_mex = get_obs_mex(self,dyn_obs, bounds)
            if ~iscell(dyn_obs)
                dyn_obs = {dyn_obs};
            end

            if numel(dyn_obs) < 2
                error('Invalid dyn_obs format. Expected at least 2 elements.');
            end
            all_pts = dyn_obs{1};
            all_vels = dyn_obs{2};
            obj_mex = [];
            n_obs = length(all_vels);
            for dyn_obs_idx = 1:n_obs
                dyn_obs_pts_start_idx = ((dyn_obs_idx-1) * 6) + 1;
                curr_pts = all_pts(:,dyn_obs_pts_start_idx:dyn_obs_pts_start_idx+3);
                deltas = max(curr_pts,[],2) - min(curr_pts,[],2);
                means = mean(curr_pts, 2);
                dyn_c_x = means(1);
                dyn_c_y = means(2);
                dyn_length = deltas(1);
                dyn_width = deltas(2);
                dyn_velocity = all_vels(dyn_obs_idx);
                dyn_heading_rad = 0;
                obj_mex(:,end+1) = [dyn_c_x; 
                                    dyn_c_y; 
                                    dyn_heading_rad;
                                    dyn_velocity;
                                    dyn_length;
                                    dyn_width];
            end
            xlo = bounds(1) ; xhi = bounds(2) ;
            ylo = bounds(3) ; yhi = bounds(4) ;
            dx = xhi - xlo;
            dy = yhi - ylo;
            x_c = mean([xlo, xhi]);
            y_c = mean([ylo, yhi]);
            b_thick = 0.01;
            b_half_thick = b_thick / 2.0;
        
            % Top
            obj_mex(:,end+1) = [x_c; yhi+b_half_thick; 0; 0; dx; b_thick];
            % Bottom
            obj_mex(:,end+1) = [x_c; ylo-b_half_thick; 0; 0; dx; b_thick];
            % Right
            obj_mex(:,end+1) = [xhi+b_half_thick; y_c; 0; 0; b_thick; dy];
            % Left
            obj_mex(:,end+1) = [xlo-b_half_thick; y_c; 0; 0; b_thick; dy];
        end


       %Generate constraints for vehrs
       function constraints = GenerateConstraints(self, zono, obs_info)
            num_obs_gens = 2;
            num_out_zono_gens = numel(zono.generatorLength) + 2;
            
            if size(obs_info, 2) < 1 || num_out_zono_gens < 1
                constraints = Constraints();
                return;
            end
            
            total_d_size = num_out_zono_gens;
            total_c_size = 2 * num_out_zono_gens;
            delta_d_arr = zeros(1, total_d_size);
            c_arr = zeros(1, total_c_size);
            
            % Compute initial values.
            max_r_without_obs = num_out_zono_gens - num_obs_gens;
            
            % delta_d(r, c) = abs(C(r,:) * G(:,c))
            for r = 1:max_r_without_obs
                delta_d_arr(r) = 0;
                
                % C(r,:) = Normalize([-G(1, r); G(2, r)]), one-indexed
                G = zono.generators;
                c_r0 = -G(1,r);
                c_r1 = G(2,r);
                norm_factor = norm([c_r0, c_r1]);
                c_r0 = c_r0 / norm_factor;
                c_r1 = c_r1 / norm_factor;
                
                % Save to C array
                c_arr(2*r) = c_r0;
                c_arr(2*r-1) = c_r1;
                
                for c = 1:num_out_zono_gens - num_obs_gens
                    g_c0 = G(2,c);
                    g_c1 = G(1,c);
                    delta_d_arr(r) = delta_d_arr(r) + abs((c_r0 * g_c0) + (c_r1 * g_c1));
                end
            end
            
            constraints.delta_d_arr_ = delta_d_arr;
            constraints.c_arr_ = c_arr;
       end

    end
end
    

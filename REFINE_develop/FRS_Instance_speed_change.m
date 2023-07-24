

classdef FRS_Instance_speed_change < rtd.planner.reachsets.ReachSetInstance
    properties
        Vehrs 
        constraints
        num_parameters
        worldinfo
        manu_type
    end

    properties
        input_range = [-1.0, 1.0]
        output_range = [-1.0, 1.0]
    end

    methods
        function self = FRS_Instance_speed_change(Vehrs,worldinfo,manu_type)
           
            %passing values to self
            self.Vehrs = Vehrs; %around 300
            self.num_parameters = 2;
            self.worldinfo = worldinfo;
            self.manu_type = manu_type;
        end
        
        
        function nlconFunction = genNLConstraint(self, worldState)%genNl constraint called trajopt
            vehrs = self.Vehrs;
            num_zonotopes = numel(vehrs);
            constraints = cell(1, num_zonotopes);
            nlconFunction = [];
            if (iscell(vehrs))
                for each_zono = 1:num_zonotopes
                    disp(class(vehrs))
                    zono = vehrs{each_zono};
                    world_info=self.worldinfo;
                    
                    obs_info = get_obs_mex(self,world_info.dyn_obstacles,world_info.bounds);
                    obs_c = obs_info(1:2, 1);
                    for z = 1:length(zono)
                        zc = zono{z}.center;
                        zg = zono{z}.generators;
                        ego_c = zc(1:2,1);
                        obs_G = obs_info(1:2, 2:end);
                        ego_G = zg(1:2,2:end);
            
                        
                        % Pad obs_c with zeros if necessary
                        if size(obs_c, 2) < size(ego_c, 2)
                            obs_c = padarray(obs_c, [0 size(ego_c, 2) - size(obs_c, 2)], 'post');
                        elseif size(obs_c, 2) > size(ego_c, 2)
                            ego_c = padarray(ego_c, [0 size(obs_c, 2) - size(ego_c, 2)], 'post');
                        end
                        
                        % Pad obs_G with zeros if necessary
                        if size(obs_G, 2) < size(ego_G, 2)
                            obs_G = padarray(obs_G, [0 size(ego_G, 2) - size(obs_G, 2)], 'post');
                        elseif size(obs_G, 2) > size(ego_G, 2)
                            ego_G = padarray(ego_G, [0 size(obs_G, 2) - size(ego_G, 2)], 'post');
                        end
            
                        
                        [PA, Pb] = armour.pz_roahm.polytope_PH([obs_c - ego_c, ego_G, obs_G]); 
        
                       nlconFunction = @(k) self.eval_constraints(k,PA,Pb);
                    end
                    
                end
            end
            
        end


         %helper function for function call
         %check
       function [h, h_eq, grad_h, grad_heq] = eval_constraints(self, k, Pa, Pb)

            h = Pa*k - Pb;
            h_eq = [];
            grad_h = Pa';
            grad_heq = [];
       end

       

    end

    methods (Access = private)

        %generate the obs_info
        function obj_mex = get_obs_mex(self,dyn_obs,bounds)
           
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

      
    end
end
    

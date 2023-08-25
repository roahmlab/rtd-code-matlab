
classdef FRS_Instance < rtd.planner.reachsets.ReachSetInstance
    properties
        Vehrs 
        constraints
        num_parameters
        worldinfo
        manu_type
        worldState
    end

    properties
        input_range = [-1.0, 1.0]
        output_range = [-1.0, 1.0]
    end

    methods
        function self = FRS_Instance(Vehrs,worldinfo,manu_type)
           
            %passing values to self
            self.Vehrs = Vehrs; 
            self.num_parameters = 2;
            self.worldinfo = worldinfo;
            self.manu_type = manu_type;

        end
        
        
        function nlconFunction = genNLConstraint(self, worldState)%genNl constraint called trajopt
            vehrs = self.Vehrs;
            self.worldState = worldState;

            nlconFunction = [];
                    zono = vehrs;
                    world_info=self.worldinfo;
                    obs_info = get_obs_mex(self,world_info.dyn_obstacles,world_info.bounds);
                    obs_c = obs_info(1:2, 1);

                    %slice at u0,v0,r0
                    [z1,id1] = zonotope_slice(zono,[7;8;9],[zono.Z(7,1);zono.Z(8,1);zono.Z(9,1)]);


                    if strcmp(self.manu_type,'speed_change')
                        [z2,id2] = zonotope_slice(z1,[11],[zono.Z(11,1)]);
                        slice_gen = [zono.Z(1,id2+1) 0;zono.Z(2,id2+1) 0];
                    elseif strcmp(self.manu_type,'lane_change')
                        [z2,id2] = zonotope_slice(z1,[12],[zono.Z(12,1)]);
                        slice_gen = [0 zono.Z(1,id2+1);0 zono.Z(2,id2+1)];
                    else
                        error('invalid manu_type')
                    end

                    
                    zc = z2.center;
                    zg = z2.generators;
                    ego_c = zc(1:2,1);
                    obs_G = obs_info(1:2, 2:end);
                    ego_G = zg(1:2,2:end);

                    [PA, Pb] = armour.pz_roahm.polytope_PH([obs_c - ego_c, ego_G, obs_G]);

                    PA_mat = PA *slice_gen;%x and y of Pu or Py
                    Pb_mat = Pb;

                   nlconFunction = @(k) self.eval_constraints(k,PA_mat,Pb_mat);%k is my lambda that is passed.

                   nlconFunctions = nlconFunction;
                self.constraints = nlconFunctions;
            
        end


       function [h, h_eq, grad_h, grad_heq] = eval_constraints(self, k, Pa, Pb)

            h = Pa *k- Pb;
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
    

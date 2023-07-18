

classdef FRS_Instance_speed_change 
    properties
        Vehrs 
        constraints
        num_parameters
        worldinfo
    end

    properties

        input_range = [-1.0, 1.0]
        output_range = [-1.0, 1.0]
        
    end

    methods
        function self = FRS_Instance_speed_change(Vehrs,worldinfo)
           
            %passing values to self
            self.Vehrs = Vehrs;
            self.num_parameters = 2;
            self.worldinfo = worldinfo;
        end
        
        
        function constraints = genNLConstraint(self, worldState)%genNl constraint called trajopt
            vehrs = self.Vehrs;
            num_zonotopes = numel(vehrs);
            constraints = cell(1, num_zonotopes);
            
            for each_zono = 1:num_zonotopes
                zono = vehrs{each_zono};
                world_info=self.worldinfo;
                disp('worldinfo class from instance: ')
                disp((world_info))
                % % dyn_obs = self.worldinfo(:,:);
                % % bounds = self.worldinfo(:,:);
                % disp(dyn_obs);
                % disp('bounds')
                % disp(bounds);
                obs_info = get_obs_mex(self,world_info.dyn_obstacles,world_info.bounds);
                constraints{each_zono} = self.GenerateConstraints(zono,obs_info);
            end
            self.constraints = constraints;
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

%you might have to write a separate function for generatorLength and 
       %Generate constraints for vehrs
       % function ret = FillZonosFromVehrs (vehrs)
       % 
       %      num_zonos = length(vehrs); %check length or size
       %      ret.num_zonos = num_zonos;
       % 
       %      num_out_zono_gen_arr = zeros(1,num_zonos);
       %      ret.num_out_zono_gen_arr = num_out_zono_gen_arr;
       % 
       %      cum_size_arr = zeros(1,num_zonos);
       %      ret.cum_size_arr = cum_size_arr ;
       % 
       %      for i=1:length(num_zonos) %index starts at 0 in each of the below case.
       %          ret.num_out_zono_gen_arr = vehrs.zono_sizes.at(i-1)+2;
       %      end
       %      cum_sum_num_out_zono_gen = 0;
       %      for i =1: length(num_zonos)
       %          cum_size_arr(i-1) = cum_sum_num_out_zono_gen;
       %          cum_sum_num_out_zono_gen = cum_sum_num_out_zono_gen + num_out_zono_gen_arr(i-1);
       %      end
       %      ret.cum_sum__num_out_zono_gen = cum_sum_num_out_zono_gen;
       % 
       %      zono_arr_size = 2* cum_sum_num_out_zono_gen;
       % 
       %      fir 
       % 
       % end
       function constraints = GenerateConstraints(self, zono, obs_info)

           %NEW CHANGES TO THE CODE.
            obs_c = obs_info(1:2, 1);
            ego_c = zono{1}.center;
            obs_G = obs_info(1:2, 2:end);
            ego_G = zono{1}.generators;
            
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
            
            [PA, Pb] = polytope_PH(obs_c - ego_c, [ego_G obs_G]);
            constraint_a_mat = PA * slice_generators;
            constraint_b_mat = Pb;

           



           %PREVIOUS IMPLEMENTATION
            % num_obs_gens = 2;
            % disp(obs_info.center)
            % disp((obs_info.generators))
            
            % [generatorLength,num_dims] = size(zono{1}.generators);
            % num_out_zono_gens = numel(generatorLength) + 2;
            % 
            % if size(obs_info, 2) < 1 || cc < 1
            %     constraints = struct('delta_d_arr_', [], 'c_arr_', []);
            %     return;
            % end
            % 
            % 
            % total_d_size = num_out_zono_gens;
            % total_c_size = 2 * num_out_zono_gens;
            % delta_d_arr = zeros(1, total_d_size);
            % c_arr = zeros(1, total_c_size);
            % 
            % % Compute initial values.
            % max_r_without_obs = num_out_zono_gens - num_obs_gens;
            % 
            % % delta_d(r, c) = abs(C(r,:) * G(:,c))
            % for r = 1:max_r_without_obs
            %     delta_d_arr(r) = 0;
            % 
            %     % C(r,:) = Normalize([-G(1, r); G(2, r)]), one-indexed
            %     G = zono.generators;
            %     c_r0 = -G(1,r);
            %     c_r1 = G(2,r);
            %     norm_factor = norm([c_r0, c_r1]);
            %     c_r0 = c_r0 / norm_factor;
            %     c_r1 = c_r1 / norm_factor;
            % 
            %     % Save to C array
            %     c_arr(2*r) = c_r0;
            %     c_arr(2*r-1) = c_r1;
            % 
            %     for c = 1:num_out_zono_gens - num_obs_gens
            %         g_c0 = G(2,c);
            %         g_c1 = G(1,c);
            %         delta_d_arr(r) = delta_d_arr(r) + abs((c_r0 * g_c0) + (c_r1 * g_c1));
            %     end
            % end
            % 
            % constraints = struct('delta_d_arr_', delta_d_arr, 'c_arr_', c_arr);
       
       end

    end
end
    

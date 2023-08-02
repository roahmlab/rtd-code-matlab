

classdef FRS_loader  < rtd.planner.reachsets.ReachSetGenerator
  
    properties
        robotState %states of the robot & UUID
        reachsets
        reachableSets
        manu_type
        vehrs %vehicle reachsets
        z_matrix %z matrix from vehrs
        desired_idx
        num_parameters
        worldinfo = struct
        t_plan
    end
     
    properties
        cache_max_size = 0 % Set the cache to 0 so that we do not carry forward old cache
    end

    
    methods
        
        function self = FRS_loader(file,t_plan,manu_type)

            %store in self
            self.num_parameters = 1;%num of parameters is 1 caz we consider lane change or speed change, one at a time. 
            self.manu_type = manu_type;
            self.t_plan = t_plan;

            %loading the file
            reachsets = loaders.RSLoader(file, do_caching=true, preload_data=false, preload_meta=false);
            self.reachsets = reachsets;
            idx=1;


            %extract all the reachsets for different groups and store them
            for rs1 = 1:reachsets.num_groups
                for rs2 = 1:reachsets.getGroup(rs1-1).num_groups
                    rs_zonos = reachsets.getGroup(rs1-1).getGroup(rs2-1).getZonos;
                    all_rs{idx} = rs_zonos;
                    idx = idx+1;
                end
            end

            
            for zono = 1:length(all_rs)
                for z = 1:length(all_rs{zono})
                   
                    z_matrix = all_rs{zono}{z}.Z;

                    %find desired_idx using t_plan
                    t_ = z_matrix(20,2:end);
                    t_idx = find(t_,1,"first");
                    t_last = z_matrix(20,t_idx);
                    t_start = z_matrix(20,1);
                    t_ub = t_start + t_last;
                    t_lb = abs(t_start - t_last);%time is never negative so abs is not required
                    if((t_plan<=t_ub) || (t_plan >=t_lb))
                        desired_idx(zono) = z;
                        des_robotState(zono) = entity.states.Robot_State(all_rs{zono}{z}.Z);
                        des_z_matrix{zono} = z_matrix;
                    end

                    
                    
                end
               
            end
            %store all the rs of the desired_idx only
            des_rs = all_rs{desired_idx};
            self.reachableSets = all_rs;
            self.vehrs = des_rs;
            self.desired_idx = desired_idx;
            self.z_matrix = des_z_matrix;
            self.robotState = des_robotState;

        end
        
         % Set worldinfo
         function setWorldInfo(self,worldinfo)
             self.worldinfo = worldinfo;
        end
        
    end

   
    methods (Access=protected)

        function reachableSet = generateReachableSet(self, robotState,varargin)%create an instance inside this to get constraints


            lon_search = self.reachsets.getSearchSet.getZonos;
            for idx1=1:length(lon_search)
                if lon_search{idx1}.in(robotState(4))
                    break;
                end
            end
            
            Mega_group = self.reachsets.getGroup(idx1-1); % Because matlab is 1-indexed, we need to subtract 1
            lat_head_search = Mega_group.getSearchSet.getZonos;
            reachableSet = struct('id', [], 'rs', []);

            for idx2 = 1:length(lat_head_search)
                lat_head_center = lat_head_search{idx2}.Z(8:9,1);
                lat_head_gen = sum(lat_head_search{idx2}.Z(8:9,2:end), 2);
                lat_head_low = lat_head_center - lat_head_gen;
                lat_head_high = lat_head_center + lat_head_gen;
                if lat_head_low(1) <= robotState(5) && robotState(5) <= lat_head_high(1) ...
                   && lat_head_low(2) <= robotState(6) && robotState(6) <= lat_head_high(2)
                    rs(idx2).id = idx2;
                    rs(idx2).rs = reachsets.FRS_Instance(Mega_group.getGroup(idx2-1).getZonos,self.worldinfo,self.manu_type);
                    
                    for z = 1:length(Mega_group.getGroup(idx2-1).getZonos)
                        z_matrix = Mega_group.getGroup(idx2-1).getZonos{z}.Z;
    
                        %find desired_idx using t_plan
                        t_ = z_matrix(20,2:end);
                        t_idx = find(t_,1,"first");
                        t_last = z_matrix(20,t_idx);
                        t_start = z_matrix(20,1);
                        t_ub = t_start + t_last;
                        t_lb = abs(t_start - t_last);%time is never negative so abs is not required
                        if((self.t_plan<=t_ub) || (self.t_plan >=t_lb))
                            desired_idx_ = z;
                        end
                    end
                    rs(idx2).desired_idx = desired_idx_;
                    
                end
            end
            reachableSet = rs;

            disp('ReachSets Generated.')
            

        end

   end
end










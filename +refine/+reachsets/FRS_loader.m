

classdef FRS_loader  < rtd.planner.reachsets.ReachSetGenerator
  
    properties
        robotState %states of the robot & UUID
        reachsets
        reachableSets
        manu_type
        vehrs %vehicle reachsets
        z_matrix %z matrix from vehrs
        rs_speed
        rs_lane
        desired_idx
        num_parameters
        worldinfo = struct
        worldState
        t_plan
    end
     
    properties
        cache_max_size = 0 % Set the cache to 0 so that we do not carry forward old cache
    end

    
    methods
        
        function self = FRS_loader(file,t_plan,manu_type)

            %store in self
            self.num_parameters = 2;%num of parameters is 1 caz we consider lane change or speed change, one at a time. 
            self.manu_type = manu_type;
            self.t_plan = t_plan;

            %loading the file
            reachsets = loaders.RSLoader(file, do_caching=true, preload_data=true, preload_meta=false);
            self.reachsets = reachsets;
            idx=1;


            for rs1 = 1:reachsets.num_groups
                for rs2 = 1:reachsets.getGroup(rs1-1).num_groups
                    rs_zonos = reachsets.getGroup(rs1-1).getGroup(rs2-1).getZonos;
                    all_rs{idx} = rs_zonos;
                    idx = idx+1;
                    

                end
            end

            new_rs = all_rs;
            for zono = 1:length(all_rs)
                for z = 1:length(all_rs{zono})
                    z_matrix = all_rs{zono}{z}.Z;
                    robotState(zono) = refine.entity.states.Robot_State(all_rs{zono}{z}.Z); 
                    if strcmp(manu_type,'lane_change')
                        mirror =-1;
                        z_matrix_ = z_matrix;
                        z_matrix_(2,:) = z_matrix(2,:) * mirror;
                        z_matrix_(3,:) = z_matrix(3,:) * mirror;
                        z_matrix_(5,:) = z_matrix(5,:) * mirror;
                        z_matrix_(6,:) = z_matrix(6,:) * mirror;
                        z_matrix_(8,:) = z_matrix(8,:) * mirror;
                        z_matrix_(9,:) = z_matrix(9,:) * mirror;
                        z_matrix_(12,:) = z_matrix(12,:) * mirror;

                        new_entry.Z = z_matrix_;
                        new_rs{zono}{end+1} = new_entry;
                    else
                        new_rs = all_rs;
                    end

                    
                end
            end
            
            %store all the rs of the desired_idx only
            self.reachableSets = new_rs;%vehrs =reachableSets
            self.vehrs = new_rs;%either remove reachableSets or vehrs from all files
            self.z_matrix = z_matrix;
            self.robotState = robotState;

        end
        
         % Set worldinfo
        function setWorldInfo(self,worldinfo)
             self.worldinfo = worldinfo;
        end
    end
        
        
    

   
    methods (Access=protected)

        function reachableSet = generateReachableSet(self, robotState,varargin)%create an instance inside this to get constraints

            self.desired_idx = 0;
            rs = struct('id', double.empty(), 'rs', struct, 'num_instances', 0); % format expected by RtdTrajOpt
            lon_search = self.reachsets.getSearchSet.getZonos;

            for idx1=1:length(lon_search)
                if lon_search{idx1}.in(robotState(4))
                    break;
                end
            end

            Mega_group = self.reachsets.getGroup(idx1-1); % Because matlab is 1-indexed, we need to subtract 1
            lat_head_search = Mega_group.getSearchSet.getZonos;
            num_instances = 0;
            for idx2 = 1:length(lat_head_search)

                lat_head_center = lat_head_search{idx2}.Z(7:8,1);
                lat_head_gen_idx1 = find(lat_head_search{idx2}.Z(7, 2:end) ~= 0, 1, 'first');
                lat_head_gen_idx2 = find(lat_head_search{idx2}.Z(8, 2:end) ~= 0, 1, 'first');
                lat_head_high_7 = lat_head_center(1) + abs(lat_head_search{idx2}.Z(7,lat_head_gen_idx1+1));
                lat_head_low_7 = lat_head_center(1) - abs(lat_head_search{idx2}.Z(7,lat_head_gen_idx1+1));
                lat_head_high_8 = lat_head_center(2) + abs(lat_head_search{idx2}.Z(8,lat_head_gen_idx2+1));
                lat_head_low_8 = lat_head_center(2) - abs(lat_head_search{idx2}.Z(8,lat_head_gen_idx2+1));
                desired_idx_ = 0;

                %if the speed change and lane change initial conditions are
                %the same, write it in one loop

                 if and((lat_head_low_7 <= robotState(4)),(robotState(4) <= lat_head_high_7 )) && strcmp(self.manu_type, 'speed_change')

                        num_instances = num_instances +1;
                       
                        for z = 1:length(Mega_group.getGroup(idx2-1).getZonos)
                            z_matrix = Mega_group.getGroup(idx2-1).getZonos{z}.Z;
                            
                            %find desired_idx using t_plan
                            t_ = z_matrix(20,2:end);
                            t_idx = find(t_,1,"first")+1;
                            t_last = z_matrix(20,t_idx);
                            t_start = z_matrix(20,1);
                            t_ub = t_start + t_last;
                            t_lb = abs(t_last - t_start);%time is never negative so abs is not required
                            if((self.t_plan<=t_ub) && (self.t_plan >=t_lb))
                                desired_idx_ = z;
                                self.desired_idx(idx2) = desired_idx_;
                            end
                          
                            
                        end

                        
                   
                elseif and((lat_head_low_7 <= robotState(4)),(robotState(4) <= lat_head_high_7 ))&& and((lat_head_low_8 <= robotState(5)),(robotState(5) <= lat_head_high_8)) && strcmp(self.manu_type,'lane_change')
                        num_instances = num_instances +1;
                     
                        
                        for z = 1:length(Mega_group.getGroup(idx2-1).getZonos)
                            z_matrix = Mega_group.getGroup(idx2-1).getZonos{z}.Z;
                            
                            %find desired_idx using t_plan
                            t_ = z_matrix(20,2:end);
                            t_idx = find(t_,1,"first")+1;
                            t_last = z_matrix(20,t_idx);
                            t_start = z_matrix(20,1);
                            t_ub = t_start + t_last;
                            t_lb = abs(t_last - t_start);%time is never negative so abs is not required
                            if((self.t_plan<=t_ub) && (self.t_plan >=t_lb))
                                desired_idx_ = z;
                                self.desired_idx(idx2) = desired_idx_;
                            end
                            
                        end


                 else
                     disp('No condition met')
                        
                        
                 end
                    

                rs(idx2).num_instances = num_instances;
            
            end
            

            if (self.desired_idx~=0)
                for idx = 1:length(self.desired_idx)
                        rs_ = refine.reachsets.FRS_Instance(Mega_group.getGroup(idx-1).getZonos{desired_idx_},self.worldinfo,self.manu_type);
                        rs(idx).rs = rs_;
                        rs(idx).id = idx;
                        self.worldState = rs_.worldState;
                end

            end
            if (num_instances==0)
                disp('rs is empty for this iteration')
                reachableSet = [];
            else
                reachableSet = rs;
            end
            

            disp('ReachSets Generated.')
            

        end

   
    end
end
    

    




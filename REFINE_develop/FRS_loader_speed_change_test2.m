
%DO NOT call the other functions inside
%IT was done only for testing
%CHECK FOR SPEED/DIRECTION/LANE --> DONE

classdef FRS_loader_speed_change_test2  < rtd.planner.reachsets.ReachSetGenerator
  
    properties
        robotState %states of the robot & UUID
        vehrs %vehicle reachsets
        z_matrix %z matrix from vehrs
        desired_idx
    end
     
    properties
        cache_max_size = 0 % Set the cache to 0 so that we do not carry forward old cache
    end

    
    methods
        
        function self = FRS_loader_speed_change_test2(file,t_plan)

            %loading the file
            reachsets = loaders.RSLoader(file, do_caching=true, preload_data=false, preload_meta=false);
                   
            %set the first_search and t_plan
            first_search = reachsets.getSearchSet.getZonos;
            % t_plan = 25;
             % min_diff = inf;
            for k = 1:length(first_search)
                    diff = abs(t_plan - sum(first_search{k}.Z));
                    if diff == 0
                        % min_diff = diff;
                        desired_idx(k) = k;  % store the index with the minimum difference
                        
                    end
                    
             end
            self.desired_idx = desired_idx;


            %for each vehicle reachset
            for rs1 = 1:reachsets.num_groups
                
                for rs2 = reachsets.getGroup(rs1-1).num_groups
                    % desired_idx = zeros(reachsets.num_groups, reachsets.getGroup(rs2-1).num_groups);
                    %need two loops one for getGroup1 and getGroup2
                    group = reachsets.getGroup(rs1-1).getGroup(rs2-1);
                    vehrs{rs2} = group.getZonos();%reachsets
                    %something seems off in the below code
                    vehrs{rs1} = vehrs{rs2}(desired_idx(desired_idx ~= 0));
                    zonos = vehrs{rs2};%
                    total_num_states = length(zonos);
                    
    
                     %set the dimensions
                    robot_state_list = cell(1,total_num_states);
                    z_matrix_list = cell(1,total_num_states);
                                   
                    
                    for i=1:total_num_states
                        % zonos = vehrs{r2}
                        z_matrix_ = zonos{i}.Z;
                        robot_state_list{i} = Robot_State(z_matrix_);
                        z_matrix_list{i} = z_matrix_;
    
                    end
    
                    
                   
                       
                    
                end
                 % min_diff = inf;
                %  for k = 1:length(first_search)
                %     diff = abs(t_plan - sum(first_search{k}.Z));
                %     if diff < min_diff
                %         min_diff = diff;
                %         desired_idx(rs1) = k;  % store the index with the minimum difference
                %     end
                % end
                Robot_state{rs1} = robot_state_list{:};%check the robot state the way it is stored
                z_matrix{rs1} = z_matrix_list;
                vehrs_list{rs1} = vehrs;
            end

            %store all values in self
            self.z_matrix = z_matrix;
            self.robotState = Robot_state;
            self.vehrs = vehrs_list;
            self.desired_idx = desired_idx;

            %FOR TESTING ONLY
            % rs = generateReachableSetWrapper(self,self.robotState);

        end
        
         % Wrapper method to call generateReachableSet ONLY FOR TESTING
        function reachableSet = generateReachableSetWrapper(self, robotState,world_state, varargin)
            reachableSet = self.generateReachableSet(robotState, world_state,varargin{:});
            disp('done rs')
        end
        
    end

   
    methods (Access=protected)

        function reachableSet = generateReachableSet(self, robotState,world_state,varargin)%create an instance inside this to get constraints

            
            disp('Generating FRS')

            %looping over all the reachsets
            for i = 1:length(self.vehrs)
                
                rs_ = struct();
                rs_ = test2_FRS_Instance_speed_change(self.vehrs{i});
                reachableSet.rs{i} = rs_;
                reachableSet.id{i} = i;
               
            end

            disp('Reachable Sets generated.');
            disp(reachableSet)
            

        end

   end
end









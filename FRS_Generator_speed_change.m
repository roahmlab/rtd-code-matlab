
classdef FRS_Generator_speed_change  < rtd.planner.reachsets.ReachSetGenerator
  
    properties
        robotState %states of the robot & UUID
    end
     
    properties
        cache_max_size = 100 % Set the desired cache maximum size
        z_matrix
        vehrs
        constraints
        generate_frs
        reach_sets_from_file
    end

    
    methods
        function self = FRS_Generator_speed_change()
            self.reach_sets_from_file = self.generate_ReachableSet('converted_Au_frs.h5');
            disp('Done with FRS generation.');
        end
    end

    methods(Access=protected)
      
        function reachsets = generate_ReachableSet(self,file) 

            %loading the file
            reachsets = loaders.RSLoader(file, do_caching=true, preload_data=false, preload_meta=false);
            collection_group = reachsets.getGroup(0);
            terminal_group = collection_group.getGroup(0);
            
            %extract vehrs
            vehrs_ = reachsets.getGroup(0).getGroup(0).getZonos;
            zonos = terminal_group.getZonos();
            total_num_states = length(zonos);
            for i=1:total_num_states
                z_matrix_ = zonos{i}.Z;
                RobotState = rtd.entity.states.EntityState();
                state = z_matrix_(1:6,:);
                time = z_matrix_(20,:);
                uuid = RobotState.uuid;
                Robot_State = struct('uuid', uuid, 'state', state,'time',time);
                
                %store all values
                self.z_matrix = z_matrix_;
                self.vehrs = vehrs_;
                self.robotState = Robot_State;
                constraints_ = frs_create_constraints(self,vehrs_);
                self.constraints = constraints_;
                fprintf('%d number:',i);
            end
            disp('DONE')
                      
        end


        function constraints = frs_create_constraints(self,vehrs)

            %create an instance to get the constraints
            constraints = FRS_Instance_speed_change(vehrs);
            self.constraints = constraints;
            disp('FRS instance with constraints generated.')
            
        end
    end

    %TO DO reachable set online
    methods (Access=protected)

        function reachableSet = generateReachableSet(self, robotState,varargin)%create an instance inside this to get constraints
            %get reachable set
            reachableSet = getReachableSet(self,robotState, varargin{:}, 'ignore_cache', true);
            disp(reachableSet);

            disp('Reachable Sets generated.');
            exit();

        end


   end
end









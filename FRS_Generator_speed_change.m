
classdef FRS_Generator_speed_change
  
    properties
        state %consists of x & y x(t) & y(t)
        head %heading of the vehicle h(t)
        yaw_rate %r rad/sec
        u0 %initial logitudnal speed
        v0 %initial lateral speed
        r0 %initial yaw rate
    end
     
    properties
        cache_max_size = 100; % Set the desired cache maximum size
        speed_lat %lateral speed v(t)
        speed_log %logitudnal speed u(t) 
        p_u %slice parameter for speed
        t %time
        %p_y %slice parameter for lane or direction change
    end

    
    methods
      
        function reach = generateReachableSet(file) %probably put this inside the constructor
            reachsets = loaders.RSLoader('converted_Au_frs.h5', do_caching=true, preload_data=false, preload_meta=false);
            downstream_group = reachsets.getGroup(0);
            arr_of_further_downstream_groups = reachsets.getGroup(0).getGroup();
            collection_group = reachsets.getGroup(0);
            terminal_group = collection_group.getGroup(0);
            
            vehrs = reachsets.getGroup(0).getGroup(0).getZonos;
            search_set = reachsets.getSearchSet();
            search_set = reachsets.getSearchSet().getZonos();
            zonos = terminal_group.getZonos();
            z_matrix = zonos{1}.Z;
            lat_speed = reachsets.getSearchSet().getZonos();
            brake_idx1 = terminal_group.attributes.brake_index1;
            brake_idx2 = terminal_group.attributes.brake_index2;
            num_slicing = collection_group.num_groups;%slicing info 
            
            reach = FRS_loader_speed_change(vehrs,brake_idx1,brake_idx2 ,z_matrix);
        end

        function self = FRS_loader_speed_change(vehrs,z_matrix)%pass all properties
            arguments               
              
                vehrs
                z_matrix 
                
            end
           
            Robot_State = z_matrix(1:6,:);
            constraints = test2_FRS_Instance_speed_change(vehrs);
            disp('FRS Instance completed')
           
        end
    end

    %TO DO reachable set online
   %  methods (Access=protected)
   % 
   %      function reachableSet = generateReachableSet(self, robotState, varargin)
   %          reachableSet = getReachableSet(self,robotState);
   %      end
   % 
   % 
   % end
end







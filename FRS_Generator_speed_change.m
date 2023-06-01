%constraints and parameters

classdef FRS_Generator_speed_change < rtd.planner.reachsets.ReachSetGenerator

   %load FRS data for speed change
   %load dummy variables for testing
       
    %properties contains the z property and meta data
    properties
        
        robot %car
        frsGenerator %FRS generator similar to jrsGenerator
        obs_frs_combs %A structure parameter with two fields: "maxcombs" defining the maximum number of combinations and "combs" storing the generated combinations.
        speed_lat %lateral speed v(t)
        speed_log %logitudnal speed u(t)
        state %consists of x & y x(t) & y(t)
        head %heading of the vehicle h(t)
        yaw_rate %r rad/sec
        u0 %initial logitudnal speed
        v0 %initial lateral speed
        r0 %initial yaw rate
        p_u %slice parameter for speed
        t %time
        brakeidx1
        brakeidx2
        %p_y %slice parameter for lane or direction change

                
    end
     
    %FRS speed generator constructor
    methods
        function self = FRS_Generator_speed_change(robot, frsGenerator,z_matrix ,options)%pass all properties
            arguments
                
                robot armour.ArmourAgent
                frsGenerator rtd.reachsets.ReadchSetGenerator
                z_matrix {mustBeFile} = 'Offline_Reachability_Analysis/FRSdata/ONLY_Z.m'%load
                options.obs_frs_combs(1,1) struct = struct('maxcombs', 200, 'combs', [])
                options.verboseLevel(1,1) rtd.util.types.LogLevel = "DEBUG"
                
            end
            self.robot = robot;
            self.frsGenerator = frsGenerator;
            self.obs_frs_combs = options.obs_frs_combs;
            self.speed_log = z_matrix.speed_log;
            self.speed_lat = z_matrix.speed_lat;
            self.state.x = z_matrix.x;
            self.state.y = z_matrix.y;
            self.head = z_matrix.h;
            self.yaw_rate = z_matrix.r;
            self.u0 = z_matrix.u0;
            self.v0 = z_matrix.r0;
            self.r0 = z_matrix.r0;
            self.p_u = z_matrix.p_u;
            self.t = z_matrix.t;

            if isempty(self.obs_frs_combs.combs)
                self.obs_frs_combs.combs = ...
                    generate_combinations_upto(self.obs_frs_combs.maxcombs);
            end
            self.set_vdisplevel(options.verboseLevel);
        end
    end

    methods (Access=protected)
        
        % Obtains the relevant reachable set for the robotstate provided
        % and outputs the singular instance of a reachable set.
        % Returns FRS Instance
        function reachableSet = generateReachableSet(self, robotState)
            % Computes the forward kinematics and occupancy
            
            % First get the FRS (allow the use of a cached value if it
            % exists)
            %frsInstance = self.frsGenerator.getReachableSet(robotState, ignore_cache=false);
            
            self.vdisp("Generating forward occupancy!", "INFO")

            % get forward kinematics and forward occupancy for refine
            frsInstance = self.frsGenerator.getReachableSet(robotState, ignore_cache=false); %right can be used for any robot state

            %create an instance to compute the reachable set for speed
            %change
            reachableSet = refine.Offline_Reachability_Analysis.FRS_Instance_speed_change(self.p_u,sliceableinfo,zonosliceinfo,vehRS_save,frsMega,frstotal);
        end
    end


end

% generate a bunch of combinations, store in a cell
% function [combs] = generate_combinations_upto(N)
% 
%     combs = cell(N, 1);
%     combs{1} = [1];
% 
%     for i = 2:N
%         combs{i} = combinator(i, 2, 'c');
%     end
% 
% end
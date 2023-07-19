

classdef Refine_Planner < rtd.planner.RtdPlanner & rtd.util.mixins.Options
    properties
        manu_type %maneuver type
        frs
        rs
        trajopt
        trajOptProps
        objective
        optimizationEngine
        trajectoryFactory
        waypoint
        vehrs
        desired_idx
        robotstate
    end
    methods (Static)
            function options = defaultoptions()
                options = struct();
                options.manu_type = 'speed change';
            end
        end
    methods 
        function self = Refine_Planner(trajOptProps,t_plan,AH,options)%need options as argument with manu_type
            arguments
                trajOptProps (1,1) rtd.planner.trajopt.TrajOptProps
                t_plan
                AH
                options struct
                % options.manu_type {mustBeMember(options.manu_type,{'speed change','direction change','lane change'})}
            end
           
            
            %dummy variables for testing
            waypoint1 = [10, 20];  % Example waypoint 1 [x1, y1]
            waypoint2 = [15, 25];  % Example waypoint 2 [x2, y2]
            waypoint3 = [20, 30];  % Example waypoint 3 [x3, y3]

            % Store waypoints in a cell array/dummy variables
            waypoint = {waypoint1, waypoint2, waypoint3};

            % Initialize the properties
            self.waypoint = waypoint;
            self.trajOptProps = trajOptProps;
           file = 'converted_Au_frs.h5';
           self.rs = struct;
            self.rs.frs_ = FRS_loader_speed_change(file,t_plan);%check what all has to be passed here
%             self.rs.AH =AH;
            %using the t_plan find desired_idx for each re
            frs = self.rs.frs_;
            for i = 1:numel(frs.vehrs)

                self.frs{i} = frs; %print frs
                self.vehrs = frs.vehrs;
                self.desired_idx = frs.desired_idx;
                vehrs = self.vehrs;
                Robotstate_ = frs.robotState;
                self.robotstate{i} = Robotstate_;
                trajF = Trajectory_Factory(AH);
           
                %Set up the optimization engine (optimizationEngine)
                self.optimizationEngine = rtd.planner.trajopt.FminconOptimizationEngine(trajOptProps);
               
                %objective function call
                objective = Refine_Objective(trajOptProps,vehrs,self.desired_idx);

                self.trajopt_speed{i} = rtd.planner.trajopt.RtdTrajOpt(trajOptProps, self.rs,objective, self.optimizationEngine, trajF);%pass the objective
%                 self.trajopt_lane{i} = rtd.planner.trajopt.RtdTrajOpt(trajOptProps, self.rs,objective, self.optimizationEngine, trajF);
                %take for speed change and lane change separetly
            end


        end

        %plan trajectory function
        function [trajectory,info] = planTrajectory(self,robotState,worldinfo,worldState)
            
            frs_ = self.frs;
            
          
            for i = 1:numel(self.trajopt)
               
                %generate frs instance-->generateReachableSet()
                frs_{i}.setWorldInfo(worldinfo);
                [trajectory, ~, info] = self.trajopt_speed{i}.solveTrajOpt(robotState, worldState, self.waypoint);
%                 [trajectory, ~, info] = self.trajopt_lane{i}.solveTrajOpt(robotState, worldState, self.waypoint);
                %best out of speed change and lane change.
            end

        end
    end

end


classdef Refine_Planner_test2 < rtd.planner.RtdPlanner & rtd.util.mixins.Options
    properties
        manu_type %maneuver type
        frs
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
                % options.input_constraints_flag = true;%where is input getting passed from/why is it used?
                % options.use_robust_input = true;%why do we need a robust input?
                options.manu_type = 'speed change';
                % options.verboseLevel = 'INFO';
            end
        end
    methods
        function self = Refine_Planner_test2(trajOptProps,t_plan)%need options as argument
            arguments
                trajOptProps (1,1) rtd.planner.trajopt.TrajOptProps
                t_plan
                % options.manu_type {mustBeMember(options.manu_type,{'speed change','direction change','lane change'})}
            end
           
            
            %dummy variables for testing
            waypoint1 = [10, 20];  % Example waypoint 1 [x1, y1]
            waypoint2 = [15, 25];  % Example waypoint 2 [x2, y2]
            waypoint3 = [20, 30];  % Example waypoint 3 [x3, y3]
            % Store waypoints in a cell array/dummy variables
            waypoint = {waypoint1, waypoint2, waypoint3};

            % Initialize the properties
            % self.manu_type = options.manu_type;
            self.waypoint = waypoint;
            self.trajOptProps = trajOptProps;
           
            frs = FRS_loader_speed_change_test2('converted_Au_frs.h5',t_plan);%check what all has to be passed here
            
            for i = 1:numel(frs.vehrs)
                %frs should be of type struct
               self.frs{i} = frs;
                self.vehrs = frs.vehrs;
                self.desired_idx = frs.desired_idx;
                vehrs = self.vehrs;
                Robotstate_ = frs.robotState(self.desired_idx ~=0);
                state = Robotstate_{1}.state;
                self.robotstate{i} = state;
                %state property u,v,r
                state_u = state.u;
                state_v = state.v;
                state_r = state.r;

                %desired x,y,h
                x_des = waypoint(:,1);
                y_des = waypoint(:,2);
                h_des = waypoint(:,3);
    
 
                au = 2.0;
                ay = 1.5;

                %initial u0 value
                u0 = state_u(1);

                %taking the initial non zero positive number for h0 value
                positive_idx = find(state.h >0);
                non_zero_idx = positive_idx(state.h(positive_idx) ~=0);
                h0 = state.h(non_zero_idx(1));

                u0_goal = 5.0;
                t0_offset = 0.2;
                
                trajF = TrajectoryFactory();
                trajectoryFactory = trajF.createTrajectory(au, ay, u0_goal, self.manu_type, u0, t0_offset, h0);
                %TAKING FIRST TRAJECTORY HERE
                self.trajectoryFactory = trajectoryFactory;
           
                %Set up the optimization engine (optimizationEngine)
                self.optimizationEngine = rtd.planner.trajopt.FminconOptimizationEngine(trajOptProps);
               
                %objective function call
                objective = Refine_Objective(trajOptProps,vehrs,self.desired_idx);

                %trajectory optimization
                self.trajopt{i} = rtd.planner.trajopt.RtdTrajOpt(trajOptProps, self.frs{i},objective, self.optimizationEngine, trajF);%pass the objective
            
            end


        end

        %plan trajectory function
        function [trajectory,info] = planTrajectory(self,robotState,worldState)
            
            
            %generate frs instance-->generateReachableSet()
            reachableSet = frs_.generateReachableSetWrapper(self, robotState, varargin);

          
            for i = 1:numel(self.trajopt)
                disp(robotState);
                [trajectory, ~, info] = self.trajopt{i}.solveTrajOpt(robotState, worldState, self.waypoint,reachableSet);
            end

        end
    end

end
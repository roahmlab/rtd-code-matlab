

classdef Refine_Planner < rtd.planner.RtdPlanner & rtd.util.mixins.Options
    
    properties
        manu_type
        frs
        rs_speed
        rs_lane
        cost_speed
        cost_lane
        AH
        trajopt_speed
        trajopt_lane
        trajOptProps
        objective
        optimizationEngine
        trajectoryFactory
        waypoint
        vehrs
        robotstate
        Nhah %NewhighwayAgentHelper
        refineplannerInstance
        worldState
    end

    methods (Static)
            function options = defaultoptions()
                options = struct();
                options.manu_type = 'speed_change';
            end
        end
    methods 
        function self = Refine_Planner(trajOptProps,worldState,HLP,Agent)%need options as argument with manu_type
            arguments
                trajOptProps (1,1) rtd.planner.trajopt.TrajOptProps
                worldState
                HLP
                Agent

            end

            file1 = 'converted_Au_frs.h5';
            file2 = 'converted_lan_frs.h5';
           

            self.rs_speed = struct;
            self.rs_lane = struct;
            self.trajOptProps = trajOptProps;
            self.worldState = worldState;
            Refine_Planner_speed_change(self,file1,self.trajOptProps,Agent,HLP,self.worldState);
            Refine_Planner_lane_change(self,file2,self.trajOptProps,Agent,HLP,self.worldState);
            

        end

        function Refine_Planner_speed_change(self,file,trajOptProps,Agent,HLP,worldState)


            %speed change
                manu_type_1='speed_change';
                self.rs_speed.frs_ = FRS_loader_speed_change(file,trajOptProps.planTime,manu_type_1);
                self.AH = NewhighwayAgentHelper(Agent,self.rs_speed.frs_ ,HLP,worldState,trajOptProps,self);
                self.Nhah = self.AH;
                self.AH.refineplannerInstance = self;
                trajF = TrajectoryFactory_pass_values(self.AH,self.rs_speed.frs_ );
                self.optimizationEngine = rtd.planner.trajopt.FminconOptimizationEngine(trajOptProps);
                self.objective = Refine_Objective(trajOptProps,self.rs_speed.frs_.vehrs);%removed desired_idx
                self.trajopt_speed = rtd.planner.trajopt.RtdTrajOpt(trajOptProps,self.rs_speed,self.objective,self.optimizationEngine,trajF);

        end

        function Refine_Planner_lane_change(self,file,trajOptProps,Agent,HLP,worldState)

            %lane change
                manu_type_2='lane_change';
                self.rs_lane.frs_ = FRS_loader_speed_change(file,trajOptProps.planTime,manu_type_2);
                self.AH = NewhighwayAgentHelper(Agent,self.rs_lane.frs_ ,HLP,worldState,trajOptProps,self);
                self.Nhah = self.AH;
                self.AH.refineplannerInstance = self;
                trajF = TrajectoryFactory_pass_values(self.AH,self.rs_lane.frs_ );
                self.optimizationEngine = rtd.planner.trajopt.FminconOptimizationEngine(trajOptProps);
                self.objective = Refine_Objective(trajOptProps,self.rs_lane.frs_.vehrs);%removed desired_idx
                self.trajopt_lane = rtd.planner.trajopt.RtdTrajOpt(trajOptProps,self.rs_lane,self.objective,self.optimizationEngine,trajF);
        end

       
        %plan trajectory function
        function [trajectory,info] = planTrajectory(self,robotState,worldinfo,waypoint)
            

                self.worldState = worldinfo.dyn_obstacles;
                self.waypoint = waypoint;
                self.rs_speed.frs_.setWorldInfo(worldinfo);
                [trajectory_speed, self.cost_speed, info_speed] = self.trajopt_speed.solveTrajOpt(robotState, self.worldState, waypoint);
                self.rs_lane.frs_.setWorldInfo(worldinfo);
                [trajectory_lane,self.cost_lane, info_lane] = self.trajopt_lane.solveTrajOpt(robotState, self.worldState, waypoint);
                
                % Check for non-empty results and set comparison flags
                is_cost_speed_empty = logical(isempty(self.cost_speed));
                is_cost_lane_empty = logical(isempty(self.cost_lane));
                cost_speed_is_less_than_cost_lane = ~is_cost_speed_empty && ~is_cost_lane_empty && (self.cost_speed < self.cost_lane);
                cost_speed_is_greater_than_cost_lane = ~is_cost_speed_empty && ~is_cost_lane_empty && (self.cost_speed > self.cost_lane);

                % Perform the comparisons
                if is_cost_lane_empty || cost_speed_is_less_than_cost_lane
                    trajectory = trajectory_speed;
                    info = info_speed;
                    self.manu_type = 'speed_change';
                elseif is_cost_speed_empty || cost_speed_is_greater_than_cost_lane
                    trajectory = trajectory_lane;
                    info = info_lane;
                    self.manu_type = 'lane_change';
                else
                    error('No cost function.');
                end



        end
    end
end




classdef Refine_Planner < rtd.planner.RtdPlanner & rtd.util.mixins.Options
    
    properties
        manu_type
        frs
        rs_speed
        rs_lane
        cost_speed
        cost_lane
        AH_speed
        AH_lane
        trajopt_speed
        trajopt_lane
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

            self.worldState = worldState;
           
            file1 = 'converted_Au_frs.h5';
            Refine_Planner_speed_change(self,file1,trajOptProps,Agent,HLP,worldState);

            file2 = 'converted_lan_frs.h5';
            Refine_Planner_lane_change(self,file2,trajOptProps,Agent,HLP,worldState);
        
      
        end


        function Refine_Planner_speed_change(self,file,trajOptProps,Agent,HLP,worldState)
            rs_speed_ = struct;
            

            %speed change
                
                rs_speed_.frs_ = refine.reachsets.FRS_loader(file,trajOptProps.planTime,'speed_change');
                self.AH_speed = refine.NewhighwayAgentHelper(Agent,rs_speed_.frs_ ,HLP,worldState,trajOptProps,self);
                self.Nhah = self.AH_speed;
                self.AH_speed.refineplannerInstance = self;
                trajF = refine.trajectory.TrajectoryFactory_pass_values(self.AH_speed,rs_speed_.frs_ ,'speed_change',trajOptProps);
                
                self.optimizationEngine = rtd.planner.trajopt.FminconOptimizationEngine(trajOptProps);
                self.objective = refine.trajopt.Refine_Objective(trajOptProps,rs_speed_.frs_.vehrs,'speed_change');%removed desired_idx
                self.trajopt_speed = rtd.planner.trajopt.RtdTrajOpt(trajOptProps,rs_speed_,self.objective,self.optimizationEngine,trajF);
                
                self.rs_speed = rs_speed_;

        end

        function Refine_Planner_lane_change(self,file,trajOptProps,Agent,HLP,worldState)

            rs_lane_ = struct;
   
            rs_lane_.frs_ = refine.reachsets.FRS_loader(file,trajOptProps.planTime,'lane_change');
            self.AH_lane = refine.NewhighwayAgentHelper(Agent,rs_lane_.frs_,HLP,worldState,trajOptProps,self);
            self.Nhah = self.AH_lane;
            self.AH_lane.refineplannerInstance = self;
            trajF = refine.trajectory.TrajectoryFactory_pass_values(self.AH_lane,rs_lane_.frs_ ,'lane_change',trajOptProps);
            self.optimizationEngine = rtd.planner.trajopt.FminconOptimizationEngine(trajOptProps);
            self.objective = refine.trajopt.Refine_Objective(trajOptProps,rs_lane_.frs_.vehrs,'lane_change');%removed desired_idx
            self.trajopt_lane = rtd.planner.trajopt.RtdTrajOpt(trajOptProps,rs_lane_,self.objective,self.optimizationEngine,trajF);
            self.rs_lane = rs_lane_;
        end

       
        %plan trajectory function
        function [trajectory,info] = planTrajectory(self,robotState,worldinfo,waypoint)
            

                self.worldState = worldinfo.dyn_obstacles;

                %mirror waypoint for lane_change manu type
                if strcmp(self.manu_type,'lane_change')
                    waypoint_ = -1 * waypoint;
                    waypoint = [waypoint, waypoint_];
                end
                self.waypoint = waypoint;
                self.rs_lane.frs_.setWorldInfo(worldinfo);
                [trajectory_lane,self.cost_lane, info_lane] = self.trajopt_lane.solveTrajOpt(robotState, self.worldState, waypoint);
                self.rs_speed.frs_.setWorldInfo(worldinfo);
                [trajectory_speed, self.cost_speed, info_speed] = self.trajopt_speed.solveTrajOpt(robotState, self.worldState, waypoint);

                %Check for non-empty results and set comparison flags
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



classdef Refine_Planner < rtd.planner.RtdPlanner & rtd.util.mixins.Options
    properties
        manu_type %maneuver type
        frs
        rs_speed
        rs_lane
        AH
%         rs
%         rs1%speed change
%         rs2%lane change
        trajopt
        trajopt_speed
        trajopt_lane
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
        function self = Refine_Planner(trajOptProps,AH)%need options as argument with manu_type
            arguments
                trajOptProps (1,1) rtd.planner.trajopt.TrajOptProps
                
                AH
       end
           

            self.trajOptProps = trajOptProps;
            file = 'converted_Au_frs.h5';

            %NEW ADDITIONS
            self.AH = AH;
            self.rs_speed = struct;
            self.rs_lane = struct;

            %speed change
            manu_type='speed_change';
            self.rs_speed.frs_ = FRS_loader_speed_change(file,trajOptProps.planTime,manu_type);
            trajF = TrajectoryFactory_pass_values(AH);
            self.optimizationEngine = rtd.planner.trajopt.FminconOptimizationEngine(trajOptProps);
            objective = Refine_Objective(trajOptProps,self.rs_speed.frs_.vehrs,self.desired_idx);
            self.trajopt_speed = rtd.planner.trajopt.RtdTrajOpt(trajOptProps,self.rs_speed,objective,self.optimizationEngine,trajF);

            %lane change
            manu_type='lane_change';
            self.rs_lane.frs_ = FRS_loader_speed_change(file,trajOptProps.planTime,manu_type);
%             trajF = TrajectoryFactory_pass_values(AH);
            self.optimizationEngine = rtd.planner.trajopt.FminconOptimizationEngine(trajOptProps);
            objective = Refine_Objective(trajOptProps,self.rs_lane.frs_.vehrs,self.desired_idx);
            self.trajopt_lane = rtd.planner.trajopt.RtdTrajOpt(trajOptProps,self.rs_lane,objective,self.optimizationEngine,trajF);


        end

        %plan trajectory function
        function [trajectory,info] = planTrajectory(self,robotState,worldinfo,worldState)
            self.rs_speed.frs_.setWorldInfo(worldinfo);
            self.rs_lane.frs_.setWorldInfo(worldinfo);
            self.waypoint = self.AH.waypoint;

                    [trajectory_speed, cost_speed, info_speed] = self.trajopt_speed.solveTrajOpt(robotState, worldState, self.waypoint); %HERE PASSING THE INITIAL GUESS -> no need of initiAL guess
                    [trajectory_lane,cost_lane, info_lane] = self.trajopt_lane.solveTrajOpt(robotState, worldState, self.waypoint);

                    if(cost_speed < cost_lane)
                        trajectory = trajectory_speed;
                        info = info_speed;
                    else
                        trajectory = trajectory_lane;
                        info = info_lane;

                    end



        end
    end

end
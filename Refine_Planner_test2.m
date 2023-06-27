

classdef Refine_Planner_test2 < rtd.planner.RtdPlanner & rtd.util.mixins.Options
    properties
        manu_type %maneuver type
        frs
        trajopt
        objective
        optimizationEngine
        trajectoryFactory
    end
    methods (Static)
            function options = defaultoptions()
                options = struct();
                options.input_constraints_flag = true;%where is input getting passed from/why is it used?
                options.use_robust_input = true;%why do we need a robust input?
                options.manu_type = 'speed change';
                options.verboseLevel = 'INFO';
            end
        end
    methods
        function self = Refine_Planner_test2(vehrs_,frs,state,desired_idx_,waypoint,trajOptProps)
            arguments
                vehrs_ 
                frs
                state
                desired_idx_ %time in the zono
                waypoint 
                trajOptProps (1,1) rtd.planner.trajopt.TrajOptProps
            end
           
            % Initialize the properties

            self.trajopt = cell(size(vehrs_));
            self.manu_type = 'speed change';
            state_ = state;
            state = state.state;
            for i = 1:numel(vehrs_)
                self.frs{i} = frs;
                vehrs = vehrs_;
                desired_idx = desired_idx_;
                state_u = state.u;
                state_v = state.v;
                state_r = state.r;
                x_des = waypoint(:,1);
                y_des = waypoint(:,2);
                h_des = waypoint(:,3);
    
                 % DUmmy variables for testing --> from the original simulation code
                trajOptProps.timeForCost = 1.0; % Set the time used for evaluation in the cost function
                trajOptProps.planTime = 0.5; % Set the time duration of the nominal plan
                trajOptProps.horizonTime = 1.0; % Set the time of the overall trajectory until stop
                trajOptProps.doTimeout = false; % Set whether or not to timeout the optimization
                trajOptProps.timeoutTime = 0.5; % Set the timeout time for the optimization
                trajOptProps.randomInit = false; % Set whether or not to randomize unknown or extra parameters
                
                %trajectory Factory
                %Dummy variables for trajectory factory
                au = 2.0;
                ay = 1.5;
                u0_goal = 5.0;
                manu_type = 'speed change';
                u0 = 1.0;
                t0_offset = 0.2;
                h0 = 0.1;
                trajF = TrajectoryFactory();
                trajectoryFactory = trajF.createTrajectory(au, ay, u0_goal, manu_type, u0, t0_offset, h0);
                self.trajectoryFactory = trajectoryFactory{1};
           
                %Set up the optimization engine (optimizationEngine)
                self.optimizationEngine = rtd.planner.trajopt.FminconOptimizationEngine(trajOptProps);
                
                %Dummy objective with cost --> change it to Lucas' cost
                %function after testing
                %writing a function for the below
                zono_center_xyh = [0;0;0];
                zono_center_u0v0r0p = [5; 0; 0; 10];
                zono_u0v0r0p_slice_gens_xyh = [1 2 3 4; 5 6 7 8; 9 10 11 12];
                zono_u0v0r0p_slice_gens_slice = [1; 0.1; 0.1; 2];
                u0v0r0 = [5.1; 0.01; 0.01];
                param_val = 11.3;
                target_pose = [1; 0.2; 0.03];
                weight_h = 3;
                weight_xy = 10;
                h_epsilon = 1.0e-6;
                xy_epsilon = 1.0e-6;

                objective = DummyObjective();
                objectiveCallback = objective.genObjective();

                % Call the objective callback function with the desired parameters
                cost = objectiveCallback({zono_center_xyh, zono_center_u0v0r0p, zono_u0v0r0p_slice_gens_xyh, ...
                    zono_u0v0r0p_slice_gens_slice, u0v0r0, param_val, target_pose, weight_h, weight_xy, h_epsilon, xy_epsilon});
                objective.cost = cost;
                self.trajopt{i} = rtd.planner.trajopt.RtdTrajOpt(trajOptProps, self.frs{i},objective , self.optimizationEngine, trajF);
            
            end


        end

        %plan trajectory function
        function [trajectory,info] = planTrajectory(self,robotState,worldState,waypoint)

            for i = 1:numel(self.trajopt)
                [trajectory, ~, info] = self.trajopt{i}.solveTrajOpt(robotState, worldState, waypoint);
            end

        end
    end

end
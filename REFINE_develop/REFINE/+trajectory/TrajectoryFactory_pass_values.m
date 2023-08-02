classdef TrajectoryFactory_pass_values  < rtd.planner.trajectory.TrajectoryFactory

    %trajectory factory pass through function

    properties
        AH
    end
    methods
        function self = TrajectoryFactory_pass_values(AH)
            self.AH = AH;
        end

        function traj = createTrajectory(self,robotState,rsInstance,parameter)

            manu_type = rsInstance.frs_.manu_type;
           
            K = [parameter(1),1,0];%returned from test_mex c++ file

            if strcmp(manu_type, 'speed_change')
                K(3) = 1;
            elseif strcmp(manu_type,'lane_change')
                K(3) = 3;
            end

            traj = K;

            x_des = self.AH.waypoint(:,1);
            
            if (self.AH.cur_t0_idx > 1 && self.AH.prev_action == 2) || (self.AH.cur_t0_idx > 2 && self.AH.prev_action == 3)|| self.AH.prev_action  == 1
                self.AH.prev_action = -1;
                self.AH.cur_t0_idx = 1;
            end

            if self.AH.prev_action ~= -1 
                K = [self.AH.saved_K(1); self.AH.saved_K(2); self.AH.cur_t0_idx ;self.AH.prev_action];
                self.AH.cur_t0_idx = self.AH.cur_t0_idx + 1;
                return
            end


            tic

            t_temp = toc;
            self.AH.solve_time_hist = [self.AH.solve_time_hist, t_temp];

            if K(end) == -1
                K = [];

            else
                type_manu = K(3);
                multiplier = 1;
                mirror_flag = 0;


                %changes
                type_text = type_manu;
                M = self.AH.zono_full.vehrs;%the frs


                if type_manu == 1
                    k = K(1);
                    FRS = M(type_text);
                    self.AH.prev_action = -1; 
                    self.AH.cur_t0_idx = 1;

                else
                    k = K(2);
                    if k<0
                        multiplier = -1;
                        mirror_flag = 1;
                    end


                    FRS = M(type_text); 
                    self.AH.prev_action = type_manu;
                    self.AH.cur_t0_idx = 2;
                    self.AH.saved_K = K;
                end
                if size(FRS,1) == 1
                    FRS = FRS';
                end


                self.AH.plot_selected_parameter_FRS(k,type_manu,FRS,mirror_flag,self.AH.agent_state,multiplier);
            
                self.AH.waypt_hist = [self.AH.waypt_hist x_des];
                self.AH.K_hist = [self.AH.K_hist k];
                self.AH.mirror_hist = [self.AH.mirror_hist mirror_flag];
                self.AH.type_manu_hist = [self.AH.type_manu_hist type_manu];
                self.AH.state_hist = [self.AH.state_hist self.AH.agent_state];
                self.AH.time_hist = [self.AH.time_hist self.AH.A.time(end)];

            end


        end
    end


end

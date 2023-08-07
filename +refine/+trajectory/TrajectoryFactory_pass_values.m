classdef TrajectoryFactory_pass_values  < rtd.planner.trajectory.TrajectoryFactory

    %trajectory factory pass through function

    properties
        AH
        frs
    end
    methods
        function self = TrajectoryFactory_pass_values(AH,frs)

                self.AH = AH;
                self.frs = frs;

        end

        function traj = createTrajectory(self,robotState,rsInstance,parameter)
            if (self.AH.cur_t0_idx > 1 && self.AH.prev_action == 2) || (self.AH.cur_t0_idx > 2 && self.AH.prev_action == 3)|| self.AH.prev_action  == 1
                self.AH.prev_action = -1;%are we using this?
                self.AH.cur_t0_idx = 1;
            end

            manu_type = rsInstance.frs_.manu_type;
            waypoint = self.AH.refineplannerInstance.waypoint;
            K = [parameter(1),parameter(2),self.AH.cur_t0_idx,0];%returned from test_mex c++ file
            rs = rsInstance.frs_.Vehrs;
            idx_11 = find(rs.Z(2:end)~=0,1,'first');
            idx_12 = find(rs.Z(2:end)~=0,1,'first');
            
            

            if strcmp(manu_type, 'speed_change')
                K(4) = 1;
                K(1) = K(1) * rs.Z(11,idx_11) + rs.Z(11,1);%scaling done here
                K(2) = 0;
                x_des = waypoint(:,1);
                self.AH.prev_action = -1; 
                self.AH.cur_t0_idx = 1;
            elseif strcmp(manu_type,'lane_change')
                K(4) = 3;
                K(1) = rs.Z(4,1);
                K(2) = K(2) * rs.Z(12,idx_12) + rs.Z(12,1);
                x_des = waypoint(:,1);
                self.AH.prev_action = 3;%are we using this?
                self.AH.cur_t0_idx = 2;
                self.AH.saved_K = K;
                K(3) = self.AH.cur_t0_idx;
                
            end

            if self.AH.prev_action ~= -1 
                K = [self.AH.saved_K(1); self.AH.saved_K(2); self.AH.cur_t0_idx ;self.AH.prev_action];
                self.AH.cur_t0_idx = self.AH.cur_t0_idx + 1;
                K(3) = self.AH.cur_t0_idx;
                traj = K;
                return
            end
            

            if self.AH.prev_action ~= -1 
                K = [self.AH.saved_K(1); self.AH.saved_K(2); self.AH.cur_t0_idx ;self.AH.prev_action];
                self.AH.cur_t0_idx = self.AH.cur_t0_idx + 1;
                traj = K;
                return
            end
            traj = K;
            

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
                M = self.AH.zono_full;%the frs


                if type_manu == 1
                    k = K(1);
                    FRS = M(type_text);
                   

                else
                    k = K(2);
                    if k<0
                        multiplier = -1;
                        mirror_flag = 1;
                    end


                    FRS = M(type_text); 
                   
                end
                if size(FRS,1) == 1
                    FRS = FRS';
                end
                self.AH.plot_selected_parameter_FRS(k,type_manu,FRS,mirror_flag,self.AH.agent_state,multiplier);
            
                self.AH.waypt_hist = [self.AH.waypt_hist x_des];
                self.AH.K_hist = [self.AH.K_hist k];
%                 self.AH.FRS_hist{end+1} = FRS;
                self.AH.mirror_hist = [self.AH.mirror_hist mirror_flag];
                self.AH.type_manu_hist = [self.AH.type_manu_hist type_manu];
                self.AH.state_hist = [self.AH.state_hist self.AH.agent_state];
                self.AH.time_hist = [self.AH.time_hist self.AH.A.time(end)];

            end
            traj = K;
            

        end

    end


end

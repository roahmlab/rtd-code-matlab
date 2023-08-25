

classdef TrajectoryFactory_pass_values  < rtd.planner.trajectory.TrajectoryFactory

    properties
        AH
        frs
        manu_type
    end
    methods
        function self = TrajectoryFactory_pass_values(AH,frs,manu_type,trajOptProps)

                self = self@rtd.planner.trajectory.TrajectoryFactory()
                self.AH = AH;
                self.frs = frs;
                self.manu_type = manu_type;
                self.trajOptProps = trajOptProps;

        end

        function traj = createTrajectory(self,robotState,rsInstance,parameter)
            K = [parameter(1),parameter(2),1,0];%returned from test_mex c++ file
            rs = rsInstance.frs_.Vehrs;
             if strcmp(self.manu_type, 'speed_change') 
                idx_11 = find(rs.Z(11,2:end) ~= 0, 1, 'first');
                K(4) = 1;
                K(1) = K(1) * rs.Z(11,idx_11+1) + rs.Z(11,1);%scaling done here
                K(2) = 0;

            elseif strcmp(self.manu_type,'lane_change')
                idx_12 = find(rs.Z(12,2:end) ~= 0, 1, 'first');
                K(4) = 3;
                K(1) = rs.Z(7,1);
                K(2) = K(2) * rs.Z(12,idx_12+1) + rs.Z(12,1);
             end

            traj = K;
            

        end

    end
end

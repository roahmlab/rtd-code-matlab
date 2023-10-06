classdef JRSInstance < rtd.planner.reachsets.ReachSetInstance
% Individual instance of a Joint Reachable Set (JRS)
%
% This class is a basic container holding the information for a single
% instance of a JRS. It is meant to be used in conjunction with the
% other generators present in this package.
%
% --- More Info ---
% Author: Adam Li (adamli@umich.edu)
% Written: 2022-10-01
% Updated: 2023-09-22 (Adam Li)
%
% See also: armour.reachsets.JRS.OnlineGeneratorBase,
% rtd.planner.reachsets.ReachSetInstance
%
% --- Revision History ---
% 2023-09-22: jrs_info field was removed parameter scaling was moved to a seperate function
%
% --- More Info ---
%

    properties
        % The range of the optimization values
        input_range = [-1.0, 1.0]
        % The range that would be mapped to for use based on this reachset
        output_range = [-1.0, 1.0]
        % The number of parameters that are being optimized from this reachset
        num_parameters = 0
        
        % Reference position
        q_des
        % Reference velocity
        dq_des
        % Reference acceleration
        ddq_des
        % Reachability for the position
        q
        % Reachability for the velocity
        dq
        % Reachability for the auxilliary velocity
        dq_a
        % Reachability for the auxilliary acceleration
        ddq_a
        % Rotatotope corresponding to q_des
        R_des
        % Transpose of the rotatotope corresponding to q_des
        R_t_des
        % Rotatotope corresponding to q
        R
        % Transpose of the rotatotope corresponding to q
        R_t
        
        % Number of joints
        n_q
        % Number of time steps
        n_t
        % Dimension id's corresponding to the parameters
        k_id
        % Name of the trajectory used to generate this reachable set
        traj_name
    end

    methods
        function setParamRange(self, param_range)
            % Sets the range of the parameters that are being optimized
            % for this reachset
            %
            % Parameters:
            %   param_range: The output range of the parameters used
            %
            arguments
                self(1,1) armour.reachsets.JRS.JRSInstance
                param_range (:,2) double
            end
            self.input_range = ones(self.num_parameters, 1) .* [-1.0, 1.0];
            self.output_range = ones(self.num_parameters, 1) .* param_range;
        end

        function setTrajName(self, traj_name)
            % Sets the name of the trajectory used to generate this
            % reachable set
            %
            % Parameters:
            %   traj_name: The name of the trajectory used
            %
            arguments
                self(1,1) armour.reachsets.JRS.JRSInstance
                traj_name {mustBeTextScalar}
            end
            self.traj_name = char(traj_name);
        end
        
        % Handles the obstacle-frs pair or similar to generate the
        % nlconstraint.
        % Returns a function handle for the nlconstraint generated
        % where the function's return type is [c, ceq, gc, gceq]
        function nlconFunction = genNLConstraint(self, worldState)
            % This is no-op, so return empty set.
            nlconFunction = [];
        end
    end
end
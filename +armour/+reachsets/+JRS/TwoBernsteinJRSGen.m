classdef TwoBernsteinJRSGen < armour.reachsets.JRS.OnlineGeneratorBase
% Generate for a trajectory composed of two 5th order bernstein polynomials
%
% This class generates a joint reachable set for a trajectory composed of
% two 5th order bernstein polynomials. The first polynomial is used to
% generate a trajectory from the current state to the start of the second
% polynomial. The second polynomial is used to generate a trajectory from
% the middle of the first polynomial to the end of the trajectory.
%
% The parameters of the first polynomial are the first half of the
% parameters, and the parameters of the second polynomial are the second
% half of the parameters.
%
% The first polynomial will be referred to as the "major" polynomial, and
% the second polynomial will be referred to as the "minor" polynomial.
%
% --- More Info ---
% Author: Adam Li (adamli@umich.edu)
% Written: 2023-09-22
% Updated: 2023-10-04 (Adam Li)
%
% See also: armour.reachsets.JRS.OnlineGeneratorBase,
% armour.reachsets.JRS.JRSInstance
%
% --- More Info ---
%

    methods (Access=protected)
        % Since the parameters are split into two parts, we need to
        % override the base class to get the correct number of parameters
        % as double the number of joints.
        function makeBaseZonos(self)
            makeBaseZonos@armour.reachsets.JRS.OnlineGeneratorBase(self, num_params=self.num_joints*2)
        end

        function reachableSet = generateReachableSet(self, robotState)
            % Generate the joint reachable set
            %
            % Arguments:
            %   robotState (rtd.entity.states.ArmRobotStateInstance): The
            %       robot state to generate the reachable set for.
            %
            % Returns:
            %   reachableSet (struct): A struct array containing the
            %       reachable set for the robot state provided, where
            %       reachableSet.rs is the reachable set and
            %       reachableSet.id is the id of the reachable set.
            %
            arguments
                self
                robotState(1,1) rtd.entity.states.ArmRobotStateInstance
            end

            self.vdisp("Generating joint reachable set!", "INFO")

            % Get the range that the parameters should cover
            param_range = [self.param_center - self.param_extents, self.param_center + self.param_extents];

            % Create the reference trajectory
            self.vdisp("Creating Reference Trajectory!", "DEBUG")
            [q_ref, qd_ref, qdd_ref] = genTwoBernsteinTrajectory( ...
                robotState.position, ...
                robotState.velocity, ...
                robotState.acceleration, ...
                self.tplan, ...
                self.tfinal, ...
                self.param_zonos, ...
                param_range, ...
                self.time_zonos);

            % Create the JRS instance from this trajectory
            jrs = self.createInstanceFromReference(q_ref, qd_ref, qdd_ref);
            jrs.setParamRange(param_range);
            jrs.setTrajName('bernstein');

            % Outputs
            reachableSet.rs = jrs;
            reachableSet.id = 1;
        end
    end
end

function [q_ref, qd_ref, qdd_ref] = genTwoBernsteinTrajectory(q0, qd0, qdd0, tplan, tfinal, base_params, param_range, time_zonos)
% Generate a trajectory composed of two 5th order bernstein polynomials
%
% This function generates a trajectory composed of two 5th order bernstein
% polynomials. The first polynomial is used to generate a trajectory from
% the current state to the start of the second polynomial, but is parameterized
% by the expected end position by the overall time horizon. The second
% polynomial is used to generate a trajectory from the middle of the first
% polynomial to the end of the trajectory.
%
% The parameters of the first polynomial are the first half of the
% parameters, and the parameters of the second polynomial are the second
% half of the parameters.
%
% Arguments:
%   q0 (double): The initial joint position of the trajectory.
%   qd0 (double): The initial joint velocity of the trajectory.
%   qdd0 (double): The initial joint acceleration of the trajectory.
%   tplan (double): The time to plan for the trajectory and the start of
%       the second polynomial.
%   tfinal (double): The overall time horizon of the trajectory.
%   base_params (cell): The base parameters of the trajectory.
%   param_range (double): The range of the parameters.
%   time_zonos (cell): The time zonotopes of the trajectory.
%
% Returns:
%   q_ref (cell): The reference position trajectory as n_t x n_q.
%   qd_ref (cell): The reference velocity trajectory as n_t x n_q.
%   qdd_ref (cell): The reference acceleration trajectory as n_t x n_q.
%
    arguments
        q0(:,1) double
        qd0(:,1) double
        qdd0(:,1) double
        tplan(1,1) double
        tfinal(1,1) double
        base_params(1,:) cell
        param_range(:,2) double
        time_zonos(1,:) cell
    end

    scale1 = 1.0 / tfinal;
    scale2 = 1.0 / (tfinal-tplan);

    % Precompute powers of time for the start of the minor bernstein
    t = tplan * scale1;
    % powers of time
    t_pow = cell(1, 6);
    t_pow{1} = 1;
    for i=2:6
        t_pow{i} = t_pow{i-1}.*t;
    end

    % Get the bernstein polynomial parameters
    num_q = length(q0);
    for i = num_q:-1:1
        q_end1{i} = q0(i) + 0.5 * (base_params{i} + 1) * (param_range(i,2) - param_range(i,1)) + param_range(i,1);
        beta = armour.legacy.match_deg5_bernstein_coefficients({q0(i); qd0(i); qdd0(i); q_end1{i}; 0; 0}, tfinal);
        alpha1{i} = armour.legacy.bernstein_to_poly(beta, 5);
        
        % Compute where the second bernstein should start
        q_plan = 0;
        qd_plan = 0;
        qdd_plan = 0;
        for k=0:5
            q_plan = q_plan + alpha1{i}{k+1}.*t_pow{k+1};
            if k > 0
                qd_plan = qd_plan + k*alpha1{i}{k+1}.*t_pow{k};
            end
            if k > 1
                qdd_plan = qdd_plan + k*(k-1)*alpha1{i}{k+1}.*t_pow{k-1};
            end
        end
        qd_plan = qd_plan * scale1;
        qdd_plan = qdd_plan * scale1^2;

        % Minor bernstein parameters (relative to the end point)
        j = i+num_q;
        q_end2{i} = q_plan + 0.5 * (base_params{j} + 1) * (param_range(j,2) - param_range(j,1)) + param_range(j,1);
        beta = armour.legacy.match_deg5_bernstein_coefficients({q_plan; qd_plan; qdd_plan; q_end2{i}; 0; 0}, tfinal-tplan);
        alpha2{i} = armour.legacy.bernstein_to_poly(beta, 5);
    end

    % Create the trajectory
    num_t = length(time_zonos);
    q_ref = num2cell(zeros(num_t, num_q));
    qd_ref = q_ref;
    qdd_ref = q_ref;
    for idx_t=1:num_t
        if time_zonos{idx_t}.c >= tfinal
            q_ref{idx_t, i} = q_end2{i};
            continue
        end
        % Select the right set of parameters to use
        if time_zonos{idx_t}.c < tplan
            scale = scale1;
            alpha = alpha1;
            % Extract time
            t = time_zonos{idx_t} * scale;
        else
            scale = scale2;
            alpha = alpha2;
            % Extract time
            t = (time_zonos{idx_t} - tplan) * scale;
        end
        % Compute
        % Precompute powers of time
        t_pow = cell(1, 6);
        t_pow{1} = 1;
        for i=2:6
            t_pow{i} = t_pow{i-1}.*t;
        end
        % Compute states for step
        for i=num_q:-1:1
            for k=0:5
                q_ref{idx_t, i} = q_ref{idx_t, i} + alpha{i}{k+1}.*t_pow{k+1};
                if k > 0
                    qd_ref{idx_t, i} = qd_ref{idx_t, i} + k*alpha{i}{k+1}.*t_pow{k};
                end
                if k > 1
                    qdd_ref{idx_t, i} = qdd_ref{idx_t, i} + k*(k-1)*alpha{i}{k+1}.*t_pow{k-1};
                end
            end
            qd_ref{idx_t, i} = qd_ref{idx_t, i} * scale;
            qdd_ref{idx_t, i} = qdd_ref{idx_t, i} * scale^2;
        end
    end
end
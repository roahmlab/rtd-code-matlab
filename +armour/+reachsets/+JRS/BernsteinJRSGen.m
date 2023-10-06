classdef BernsteinJRSGen < armour.reachsets.JRS.OnlineGeneratorBase
% Online generator for Bernstein polynomial trajectories
%
% This class is used to generate reachable sets for a robot using a
% Bernstein polynomial trajectory. This is built on top of the
% OnlineGeneratorBase class.
%
% The bernstein polynomial is parameterised by the relative end position
% of the entire trajectory. It's scaled to the time horizon of the
% reachable set.
%
% This incorporates methods previously found in create_jrs_online.m written
% by Patrick Holmes for ARMOUR. This class is an extension of the
% functionality provided by that script.
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
            [q_ref, qd_ref, qdd_ref] = genBernsteinTrajectory( ...
                robotState.position, ...
                robotState.velocity, ...
                robotState.acceleration, ...
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

function [q_ref, qd_ref, qdd_ref] = genBernsteinTrajectory(q0, qd0, qdd0, tfinal, base_params, param_range, time_zonos)
% Generate a bernstein polynomial trajectory
%
% This function generates a bernstein polynomial trajectory from the
% provided initial and final states, and the time horizon. The trajectory
% is parameterised by the relative end position of the entire trajectory.
% It's scaled to the time horizon of the reachable set.
%
% Arguments:
%   q0 (double): The initial joint position of the trajectory.
%   qd0 (double): The initial joint velocity of the trajectory.
%   qdd0 (double): The initial joint acceleration of the trajectory.
%   tfinal (double): The time horizon of the trajectory.
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
        tfinal(1,1) double
        base_params(1,:) cell
        param_range(:,2) double
        time_zonos(1,:) cell
    end

    % Get the bernstein polynomial
    num_q = length(q0);
    for i = num_q:-1:1
        q_end{i} = q0(i) + 0.5 * (base_params{i} + 1) * (param_range(i,2) - param_range(i,1)) + param_range(i,1);
        beta = armour.legacy.match_deg5_bernstein_coefficients({q0(i); qd0(i); qdd0(i); q_end{i}; 0; 0}, tfinal);
        alpha{i} = armour.legacy.bernstein_to_poly(beta, 5);
    end

    scale = 1.0 / tfinal;

    % Create the trajectory
    num_t = length(time_zonos);
    q_ref = num2cell(zeros(num_t, num_q));
    qd_ref = q_ref;
    qdd_ref = q_ref;
    for idx_t=1:num_t
        if time_zonos{idx_t}.c <= tfinal
            % Extract time
            t = time_zonos{idx_t} * scale;
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
        else
            q_ref{idx_t, i} = q_end{i};
        end
    end
end
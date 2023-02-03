classdef RtdPlanner < handle
% This class specifies the overall planner
%
% It should set up anything for any special type of RTD planner.
% Construction of a special version of this should do the following:
%
% - Construct OptimizationEngine(s) with parameters
% - Construct ReachableSets for each reachable set and trajectory type
% - Construct some Objective object for the problem at hand
% - Determine/set RtdTrajOpt parameters and create it for each trajectory type
%
% Then on each waypoint, we call for a trajectory plan using
% `planTrajectory`. That will use RTD to solve for a trajectory.
%
% --- More Info ---
% Author: Adam Li (adamli@umich.edu)
% Written: 2022-08-24
%
% See also sim.planner, sim.planner.trajopt.RtdTrajOpt
%
% --- More Info ---
%

    methods
        % Generate a trajectory for the given states and waypoint.
        %
        % Loops over each RTD_TrajOpt instance (thus, each trajectory type)
        % with the given RobotState, rtd.sim.world.WorldState, Waypoint,
        % and initial guess if wanted
        %
        % From the results, this should select the best valid Trajectory.
        % Otherwise it should return an empty or invalid trajectory which
        % will throw when attempting to set the new trajectory, ensuring
        % the old one continues - or something like that.
        %
        % Arguments:
        %   robotState: Current state of the robot
        %   worldState: Current state of the world
        %   waypoint: The waypoint to consider for the trajopt problem(s)
        %
        % Returns:
        %   Trajectory or []: A trajectory object, which may or may not be
        %   valid. If no plans are found, this may alternately return an
        %   empty value.
        %
        trajectory = planTrajectory(self, robotState, worldState, waypoint)
    end
end
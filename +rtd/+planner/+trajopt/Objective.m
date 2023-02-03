classdef Objective < handle
% Base interface for creating the objective to optimize for the trajopt problem
%
% This should be extended and used to create an objective callback function
% for the optmization engine. Any necessary transformations or special
% function calls can take place inside the object.
%
% Note:
%   Must implement `genObjective(self, robotState, waypoint,
%   reachableSets)`. See help for more details.
%
% --- More Info ---
% Author: Adam Li (adamli@umich.edu)
% Written: 2022-08-24
%
% See also genObjective, rtd.planner.trajopt,
% rtd.planner.trajopt.RtdTrajOpt, rtd.planner.trajopt.GenericArmObjective
%
% --- More Info ---
%

    methods (Abstract)
        % Generate an objective callback for use with some optimizer.
        %
        % Given the information, generate a handle for an objective
        % function with two return values, where
        % `function [cost, grad_cost] = objectiveCallback(params)`
        %
        % Note:
        %   Any timeouts should be handled by the OptimizationEngine itself
        %
        % Arguments:
        %   robotState (rtd.entity.states.EntityState): Initial state of the robot
        %   waypoint: Some goal waypoint we want to get close to
        %   reachableSets: Instances of the relevant reachable sets.
        %
        % Returns:
        %   function_handle: callback function of form `[cost, grad_cost] =
        %   objectiveCallback(params)`
        %
        objectiveCallback = genObjective(self, robotState, waypoint, reachableSets)
    end
end
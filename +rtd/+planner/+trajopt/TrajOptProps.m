classdef TrajOptProps < handle
% Struct of core trajectory optimizer properties.
%
% These are the consistent properties we use across RTD for trajectory
% optimization, and they're used all across the codebase.
%
% --- More Info ---
% Author: Adam Li (adamli@umich.edu)
% Written: 2022-10-01
% Last Revised: 2023-02-02
%
% See also rtd.planner.trajopt, rtd.planner.trajopt.RtdTrajOpt
%
% --- More Info ---
%

    properties
        % The time used for evaluation in the cost function.
        timeForCost (1,1) double = 1.0

        % The time duration of the nominal plan. Braking action should
        % occur in horizonTime-planTime.
        planTime (1,1) double = 0.5

        % The time of the overall trajectory until stop.
        horizonTime (1,1) double = 1.0

        % Whether or not the timeout the optimization if it is taking too
        % long.
        doTimeout (1,1) logical = false

        % The time at which the optimization should timeout.
        timeoutTime (1,1) double = 0.5

        % Whether or not unknown or extra parameters (slack variables or if
        % no initial guess is given) should be randomized or zero
        % initialized.
        randomInit (1,1) logical = false
    end
end
classdef ReachSetInstance < rtd.util.mixins.UUID & handle
% Base class for a single reachable set generated for some state + parameters
%
% This is just an individual instance of a reachable set. It should
% hold the necessary information to make a nonlinear constraint. If a
% generated nonlinear constraint function is not atomic (specifically, if
% it can change class properties, the cache for the respective
% :mat:class:`+rtd.+planner.+reachsets.ReachSetGenerator`
% should be disabled by setting `cache_max_size` to 0.
% 
% Note:
%   `genNLConstraint(self, worldState)` should be implemented. This should
%   return a function handle used for the optimizer as a nonlinear
%   constraint. It should return 4 outputs: [c, ceq, gc, gceq], where
%   `c <= 0` and `ceq = 0` are the constraints, and `gc` and `gceq` are the
%   gradients for the respective constraints.
%
% Note:
%   `input_range` and `num_parameters` properties must be defined.
%   `input_range` is a 2 column array with min on the left and max on the
%   right. If only one row is defined, it is expanded to `num_parameters`.
%   If more rows than `num_parameters` are defined, the extra rows are
%   considered slack variables and added to the optimization independently.
%
% --- More Info ---
% Author: Adam Li (adamli@umich.edu)
% Written: 2022-08-24
% Last Revised: 2023-01-30
%
% See also genNLConstraint, rtd.planner.reachsets.ReachSetGenerator,
% fmincon
%
% --- More Info ---
%

    properties (Abstract)
        % A 2 column array denoting the input minimum and maximums for the
        % reachable set.
        input_range (:,2) double %

        % The number of main shared parameters used by this set. Generally,
        % this should match the size of the final trajectory parameters.
        num_parameters (1,1) double %
    end
    methods (Abstract)
        % Generate the nonlinear constraint function for the provided worldState
        %
        % This function should handle the obstacle-frs pair or similar to
        % generate the nonlinear constraint.
        %
        % Arguments:
        %   worldState: The observation of the world we want to generate the constraint for
        %
        % Returns:
        %   function_handle: A function handle for the generated nlconstraint where the constraint function's return type is [c, ceq, gc, gceq]
        %
        nlconFunction = genNLConstraint(self, worldState)
    end
end
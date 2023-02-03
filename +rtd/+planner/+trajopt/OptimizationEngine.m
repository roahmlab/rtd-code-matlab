classdef OptimizationEngine < handle
% Base class for any sort of nonlinear optimizer used.
%
% This class should be used to create a parameterized instance of some
% nonlinear optimizer. For example, in MATLAB, this would be fmincon.
% 
% Note:
%   Must implement `performOptimization`. See help for more details.
%
% --- More Info ---
% Author: Adam Li (adamli@umich.edu)
% Written: 2022-08-24
% Last Revised: 2022-10-01
%
% See also performOptimization, rtd.planner.trajopt,
% rtd.planner.trajopt.RtdTrajOpt,
% rtd.planner.trajopt.FminconOptimizationEngine
%
% --- More Info ---
%
    methods (Abstract)
        % Use the given optimizer to perform the optimization
        %
        % Arguments:
        %   initialGuess: An initial guess vector used for the optimization. May not be the correct size and should be accounted for if not.
        %   objectiveCallback: A callback for the objective function of this specific optimization
        %   constraintCallback: A callback for the nonlinear constraints, where the return time is expected to be [c, ceq, gc, gceq].
        %   boundsStruct: A struct containing input and output bounds.
        %
        % Returns:
        %   [success(logical), parameters(double), cost(double)]: `success`
        %   is if the optimization was successful or didn't time out.
        %   `parameters` are the trajectory parameters to use. `cost` is
        %   the final cost for the parameters found.
        %
        [success, parameters, cost] = performOptimization(      ...
                    self,                   ...
                    initialGuess,           ...
                    objectiveCallback,      ...
                    constraintCallback,     ...
                    boundsStruct           ...
                )
    end
end
classdef OptimizationEngine < handle
    % This class should create a parameterized instance of some nonlinear
    % optimizer. For example, in MATLAB, this would be fmincon.
    methods (Abstract)
        % If the optimization was successful or didn't time out, then
        % success is true, otherwise, success is false.
        [success, parameters, cost] = performOptimization(      ...
                    self,                   ...
                    initialGuess,           ...
                    objectiveCallback,      ...
                    constraintCallback,     ...
                    boundsStruct           ...
                )
    end
end
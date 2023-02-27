classdef FminconOptimizationEngine < rtd.planner.trajopt.OptimizationEngine
% The fmincon call for MATLAB encapsulated as an OptimizationEngine
%
% This class wraps the fmincon call to the format expected for some
% OptimizationEngine. It includes our recommended default fmincon options
% from one of our projects and handles optimization timeout if requested.
%
% --- More Info ---
% Author: Adam Li (adamli@umich.edu)
% Written: 2022-10-01
% Last revised: 2022-10-11
%
% See also rtd.planner.RtdPlanner, rtd.planner.trajopt.RtdTrajOpt,
% rtd.planner.trajopt.OptimizationEngine, rtd.planner.trajopt
%
% --- More Info ---
%

    properties
        % general trajectory optimiation properties
        trajOptProps rtd.planner.trajopt.TrajOptProps %

        % Fmincon options
        options %
    end
    methods
        function self = FminconOptimizationEngine( ...
                trajOptProps, ...
                extra_options)
            % Create an instance of this object.
            %
            % This also merges any options for fmincon if provided.
            %
            % Arguments:
            %   trajOptProps (rtd.planner.trajopt.TrajOptProps)
            %   extra_options: Extra options as a struct or optimoptions for fmincon
            %
            self.trajOptProps = trajOptProps;

            % Default options
            default = optimoptions('fmincon','SpecifyConstraintGradient',true);
            if exist('options','var')
                to_merge = namedargs2cell(extra_options);
            else
                to_merge = {};
            end
            self.options = optimoptions(default,to_merge{:});
        end


        function [success, parameters, cost] = performOptimization(      ...
                    self,                   ...
                    initialGuess,           ...
                    objectiveCallback,      ...
                    constraintCallback,     ...
                    boundsStruct            ...
                )
            % Use fmincon to perform the optimization
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

            % get how many extra variables we need
            n_remainder = size(boundsStruct.param_limits,1) - length(initialGuess);
            % get bounds
            lb = boundsStruct.param_limits(:,1);
            ub = boundsStruct.param_limits(:,2);
            % Generate random values if requested, otherwise zeros for any
            % thing not in our initial guess.
            if self.trajOptProps.randomInit
                initial_extra = rtd.random.deprecation.rand_range( ...
                                        lb(end-n_remainder+1:end), ...
                                        ub(end-n_remainder+1:end));
            else
                initial_extra = zeros(n_remainder, 1);
            end
            
            % build initial guess
            initial = [initialGuess(:); initial_extra(:)];
            
            % timeout function
            opts = self.options;
            if self.trajOptProps.doTimeout
                start_tic = tic;
                stop_fcn = @(~,~,~) self.timeout_fcn(start_tic);
                opts = optimoptions(opts,"OutputFcn",[{stop_fcn}, opts.OutputFcn]);
            end
            
            % fmincon call
            [parameters, cost, exitflag, ~] = fmincon( ...
                objectiveCallback, ...
                initial, ...
                [], [], [], [], ...
                lb, ub, ...
                constraintCallback, ...
                opts);
            
            success = exitflag > 0 ;
        end

        % timeout function
        function stop = timeout_fcn(self, startTime)
            % Utlity timeout function for fmincon
            %
            % Arguments:
            %   startTime: initial time, given by `tic`
            %
            % Returns:
            %   logical: whether or not to stop now, based on trajOptProps
            %
            elapsed = toc(startTime);
            stop = elapsed > self.trajOptProps.timeoutTime;
        end
    end
end

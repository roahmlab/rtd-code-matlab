classdef FminconOptimizationEngine < OptimizationEngine
    properties
        trajOptProps
        options
        timeoutEnabled
    end
    % This class should create a parameterized instance of some nonlinear
    % optimizer. For example, in MATLAB, this would be fmincon.
    methods
        function self = FminconOptimizationEngine( ...
                trajOptProps, ...
                timeoutEnabled, ...
                options, ...
                varargin)
            self.trajOptProps = trajOptProps;
            self.timeoutEnabled = timeoutEnabled;
            default = optimoptions('fmincon','SpecifyConstraintGradient',true);
            if exist('trajectoryParams','var')
                to_merge = namedargs2cell(options);
            else
                to_merge = {};
            end
            self.options = optimoptions(default,to_merge{:});
        end
                
                
        % If the optimization was successful or didn't time out, then
        % success is true, otherwise, success is false.
        function [success, parameters, cost] = performOptimization(      ...
                    self,                   ...
                    initialGuess,           ...
                    objectiveCallback,      ...
                    constraintCallback,     ...
                    boundsStruct            ...
                )
            %P.vdisp('Running trajopt', 3)
            
            % get how many extra variables we need
            n_remainder = size(boundsStruct.param_limits,1) - length(initialGuess);
            % get bounds
            lb = boundsStruct.param_limits(:,1);
            ub = boundsStruct.param_limits(:,2);
            % Replace with rand_range
            initial_extra = rand_range(lb(end-n_remainder+1:end), ...
                                       lb(end-n_remainder+1:end));
            
            % build initial guess
            initial = [initialGuess(:); initial_extra(:)];
            
            % timeout function
            opts = self.options;
            if self.timeoutEnabled
                start_tic = tic;
                stop_fcn = @(~,~,~) self.timeout_fcn(start_tic);
                opts = optimoptions(opts,"OutputFcn",[{stop_fcn}, opts.OutputFcn]);
            end
            
            [parameters, cost, exitflag, ~] = fmincon( ...
                objectiveCallback, ...
                initial, ...
                [], [], [], [], ...
                lb, ub, ...
                constraintCallback, ...
                opts);
            
            success = exitflag > 0 ;
        end
        function stop = timeout_fcn(self, startTime)
            elapsed = toc(startTime);
            stop = elapsed > self.trajOptProps.timeout;
        end
    end
end

function n = rand_range(lo,hi)
    n = (hi-lo).*rand(size(hi)) + lo ;
end

    
    
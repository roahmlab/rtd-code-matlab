classdef FminconOptimizationEngine < OptimizationEngine
    % FminconOptimizationEngine
    % This class wraps the fmincon call to the format expected for
    % OptimizationEngin subclasses. It includes our recommended default
    % fmincon options and handles optimization timeout if requested.
    properties
        % general trajectory optimiation properties
        trajOptProps TrajOptProps
        % Fmincon options
        options
    end
    methods
        % Create an instance of this object. Merge any options for fmincon
        % if provided.
        function self = FminconOptimizationEngine( ...
                trajOptProps, ...
                options, ...
                varargin)
            self.trajOptProps = trajOptProps;

            % Default options
            default = optimoptions('fmincon','SpecifyConstraintGradient',true);
            if exist('options','var')
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
            % get how many extra variables we need
            n_remainder = size(boundsStruct.param_limits,1) - length(initialGuess);
            % get bounds
            lb = boundsStruct.param_limits(:,1);
            ub = boundsStruct.param_limits(:,2);
            % Generate random values if requested, otherwise zeros for any
            % thing not in our initial guess.
            if self.trajOptProps.randomInit
                initial_extra = rand_range(lb(end-n_remainder+1:end), ...
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
            elapsed = toc(startTime);
            stop = elapsed > self.trajOptProps.timeoutTime;
        end
    end
end

% utility to generate a random range
function n = rand_range(lo,hi)
    n = (hi-lo).*rand(size(hi)) + lo ;
end

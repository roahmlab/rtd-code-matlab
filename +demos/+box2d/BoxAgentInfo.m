classdef BoxAgentInfo < rtd.entity.components.BaseInfoComponent & rtd.util.mixins.Options & handle  
    properties
        dimension = 2
        width {mustBePositive}
        height {mustBePositive}
        color(1,3) {mustBeInRange(color,0,1,"inclusive")}
    end
    

    methods (Static)
        function options = defaultoptions()
            % Configurable options for this component
            options.width = 1;
            options.height = 1;
            options.color = [1 0 1];    % magenta
        end
    end
    

    methods
        % constructor
        function self = BoxAgentInfo(optionsStruct, options)
            arguments
                optionsStruct.optionsStruct struct = struct()
                options.width {mustBePositive}
                options.height {mustBePositive}
                options.color(1,3) {mustBeInRange(options.color,0,1,"inclusive")}
            end
            self.mergeoptions(optionsStruct.optionsStruct, options);
            
            % initialize
            self.reset();
        end
        

        % reset parameters to what was set when first instantiated
        function reset(self, optionsStruct, options)
            arguments
                self
                optionsStruct struct = struct()
                options.width {mustBePositive}
                options.height {mustBePositive}
                options.color(1,3) {mustBeInRange(options.color,0,1,"inclusive")}
            end
            options = self.mergeoptions(optionsStruct, options);
            
            % Save
            self.width = options.width;
            self.height = options.height;
            self.color = options.color;
        end
    end
end
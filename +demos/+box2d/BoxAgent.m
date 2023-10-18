classdef BoxAgent < rtd.sim.world.WorldEntity & handle
    properties
        info = demos.box2d.BoxAgentInfo.empty()
        state = rtd.entity.components.GenericEntityState.empty()
        visual
    end


    methods(Static)
        function options = defaultoptions()
            options = rtd.sim.world.WorldEntity.baseoptions();
            components.info = 'demos.box2d.BoxAgentInfo';
            components.state = 'rtd.entity.components.GenericEntityState';
            components.visual = 'demos.box2d.BoxAgentVisual';
            options.components = components;
        end
    end


    methods
        % constructor
        function self = BoxAgent(components, optionsStruct, options)
            arguments
                components.info = []
                components.state = []
                components.visual = []
                optionsStruct.optionsStruct struct = struct()
                options.components
                options.component_options
            end
            % Retrieve options from provided components
            override_options = rtd.sim.world.WorldEntity.get_componentOverrideOptions(components);

            % Merge all options
            self.mergeoptions(optionsStruct.optionsStruct, options, override_options);

            % (Re)construct all components for consistency
            self.construct_component('info');
            self.construct_component('state', self.info);
            self.construct_component('visual', self.info, self.state);

            self.reset()
        end

        
        % reset parameters to what was set when first instantiated
        function reset(self, optionsStruct, options)
            arguments
                self
                optionsStruct.optionsStruct struct = struct()
                options.components
                options.component_options
            end
            self.mergeoptions(optionsStruct.optionsStruct, options);
            
            self.reset_components()
        end
    end
end
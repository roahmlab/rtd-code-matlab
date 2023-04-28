classdef BoxAgent < rtd.sim.world.WorldEntity & handle
    properties
        info = BoxAgentInfo.empty()
        state = rtd.entity.components.GenericEntityState.empty()
        visual
    end


    methods(Static)
        function options = defaultoptions()
            options = rtd.sim.world.WorldEntity.baseoptions();
            components.info = 'BoxAgentInfo';
            components.state = 'rtd.entity.components.GenericEntityState';
            components.visual = 'BoxAgentVisual';
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
            self.mergeoptions(optionsStruct.optionsStruct, options);

            % initialize info
            if ~isempty(components.info)
                options.component_options.BoxAgentInfo = components.info.instanceOptions;
                self.construct_component('info', options=components.info.instanceOptions);
            else
                self.construct_component('info');
            end
            % initialize state
            if ~isempty(components.state)
                options.component_options.rtd.entity.components.GenericEntityState = components.state.instanceOptions;
                self.construct_component('state', self.info, components.state.instanceOptions);
            else
                self.construct_component('state', self.info);
            end
            % initialize visual
            if ~isempty(components.visual)
                options.component_options.BoxAgentVisual = components.visual.instanceOptions;
                self.construct_component('visual', self.info, self.state, components.visual.instanceOptions);
            else
                self.construct_component('visual', self.info, self.state);
            end
            self.mergeoptions(optionsStruct.optionsStruct, options);

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


    methods 
        % plot single frame
        function plot(self, options)
            arguments
                self
                options.time(1,1) double = self.visual.box_state.time(end)
                options.xlim(2,1) double = [-5 5]
                options.ylim(2,1) double = [-5 5]
            end
            xlim(options.xlim);
            ylim(options.ylim);
            self.visual.plot_at_time(options.time);
        end

        
        % animate from t = [start_time, end_time]
        function animate(self, options)
            arguments
                self
                options.start_time(1,1) double = 0;
                options.end_time(1,1) double = self.visual.box_state.time(end)
                options.speed(1,1) double = 1
                options.fps(1,1) double = 5
            end
            step_time = 1./options.fps;
            for t = options.start_time:options.speed*step_time:options.end_time
                self.plot(time=t);
                pause(step_time);
            end
        end
    end
end
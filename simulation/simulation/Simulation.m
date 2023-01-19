classdef Simulation < mixins.NamedClass & handle
    % properties
    %     world_def
    % end
    properties (Abstract)
        simulation_timestep
    end
    methods (Abstract)
        % add some object to the simulation
        add_object(self, object)
        % setup all the world
        setup(self)
        % initialize everything to start
        initialize(self)
        % Execute before the overall step
        pre_step(self)
        % Execute all the updates needed for each step
        step(self)
        % Execute after each step
        post_step(self)
        % generate the summary
        summary(self, options)
        
        % Run the lifecycle
        % Max iterations or max length is embedded in this.
        run(self, options)
        
        % Exports the world_def struct to a
        %export_world(self, filename)
        %import_world(self, filename)
        
        % saves and loads the simulation
        %save_checkpoint(self, filename)
        %load_checkpoint(self, filename)
    end
end
classdef WorldEntity < matlab.mixin.Heterogeneous & rtd.util.mixins.Options & rtd.util.mixins.NamedClass & handle
% WorldEntity is the base class for all entities in the world. It is
% responsible for managing the state and info of the entity, as well as
% providing a common interface for interacting with the entity.
%
% It is also responsible for managing the options of the entity, and
% providing a common interface for interacting with the options.
%
% --- More Info ---
% Author: Adam Li (adamli@umich.edu)
% Written: 2023-01-11
%
% See also: rtd.sim.world.WorldModel, rtd.sim.BaseSimulation,
% rtd.util.mixins.Options, rtd.util.mixins.NamedClass
%
% --- More Info ---
%

    properties (Abstract)
        % Intrinsic properties of the entity which generally do not change
        info rtd.entity.components.BaseInfoComponent

        % Internal state of the entity which changes over time
        state rtd.entity.components.BaseStateComponent
    end

    properties (Dependent)
        % UUID of the entity from the info
        uuid
    end

    methods
        function update_name(self, name)
            % Update the name of the entity and all of its components
            %
            % Arguments:
            %     name: Name of the entity
            %
            arguments
                self(1,1) rtd.sim.world.WorldEntity
                name {mustBeTextScalar}
            end

            % update the name
            self.name = name;
            
            % update options
            options = self.getoptions();
            options.name = name;
            options = self.mergeoptions(options);
            components = fieldnames(options.components).';

            % Overwrite the name
            for component_name = components
                if isprop(self.(component_name{1}), 'name')
                    self.(component_name{1}).name = name;
                end
            end
        end

        function component = construct_component(self, name, varargin)
            % Construct a component with the given name and arguments
            %
            % Arguments:
            %     name: Name of the component
            %     varargin: Arguments to pass to the component
            %
            % Returns:
            %     component: Constructed component
            %
            arguments
                self(1,1) rtd.sim.world.WorldEntity
                name {mustBeTextScalar}
            end
            arguments (Repeating)
                varargin
            end

            % Get the options for the component if they exist
            option_struct = self.instanceOptions;
            if isfield(option_struct, 'component_options') ...
                    && isfield(option_struct.component_options, name)
                % If we have options, pass them in
                nvargs = namedargs2cell(option_struct.component_options.(name));
                component = feval(option_struct.components.(name), varargin{:}, nvargs{:});
            else
                % Otherwise, just pass in the arguments
                component = feval(option_struct.components.(name), varargin{:});
            end

            self.(name) = component;
        end

        function reset_components(self)
            % Reset all components of the entity
            %
            % TODO add inclusion, exclusion options
            %

            % Get the latest options
            options = self.getoptions();

            % Get all component names
            components = fieldnames(options.components).';

            % Iterate in order
            for component_name = components
                % if we don't have options for this component, just reset
                % and continue
                if ~isfield(options.component_options, component_name)
                    self.(component_name{1}).reset();
                    continue
                end

                % Overwrite their names if we have a name for our class
                if ~isempty(options.name) ...
                        && isfield(options.component_options.(component_name{1}), 'name') ...
                        && isempty(options.component_options.(component_name{1}).name)
                    options.component_options.(component_name{1}).name = options.name;
                    
                end
                % Overwrite the verboseLevel if we have a logleveloverride
                if ~isempty(options.component_logLevelOverride) ...
                        && isfield(options.component_options.(component_name{1}), 'verboseLog')
                    options.component_options.(component_name{1}).verboseLog = options.component_logLevelOverride;
                    
                end
                self.(component_name{1}).reset(options=options.component_options.(component_name{1}));
            end
        end

        function options = getoptions(self)
            % Get the options for the entity and all of its components
            % Also performs an internal update of the options
            %
            % Returns:
            %     options (struct): Options for the entity and its components
            %

            % Update all the component options before returning.

            % Get a proposal set of options
            options = getoptions@rtd.util.mixins.Options(self);

            % Get all the component options
            options.component_options = get_componentoptions(options.components, self);

            % Merge it back into the stored options
            options = self.mergeoptions(options);
        end

    end

    % Methods for dependent properties
    methods
        function uuid = get.uuid(self)
            uuid = self.info.uuid;
        end
    end

    methods (Static)
        function options = baseoptions()
            % Base options for any world entity
            %
            % Returns:
            %     options (struct): Options for the entity and its components
            %

            options.components = struct;
            options.component_options = struct;%get_componentoptions(components);
            options.component_logLevelOverride = [];
            options.verboseLevel = 'INFO';
            options.name = '';
        end

        function options = get_componentOverrideOptions(components)
            % Gets options for provided components if they are already constructed or the default options if they aren't
            % and returns them in a partial options struct for the entity to merge in as overrides
            %
            % Arguments:
            %     components (struct): Struct of components to generate override options for
            %
            % Returns:
            %     options (struct): Partial struct of WorldEntity options for the just the components provided
            %
            arguments
                components (1,1) struct
            end

            % mark each of our preconstructed components and save them
            names = fieldnames(components).';
            components = struct2cell(components).';

            % Boolean mask for provided components
            provided_components_mask = ~cellfun(@isempty, components);
            provided_components = components(provided_components_mask);

            % Preprocess and save the names
            provided_component_classnames = cellfun(@class, provided_components, 'UniformOutput', false);
            string_mask = ismember(provided_component_classnames,{'char','string'});
            provided_component_classnames(string_mask) = provided_components(string_mask);
            provided_component_names = names(provided_components_mask);
            options.components = [provided_component_names; provided_component_classnames];
            options.components = struct(options.components{:});

            % Preprocess the instances for options
            component_handles = [provided_component_names(~string_mask); provided_components(~string_mask)];
            options.component_options = get_componentoptions(struct(component_handles{:}));
        end
    end
end

% Helper functions
function componentoptions = get_componentoptions(component_classnames, components)
    % Get the options for the provided components or call their default options
    % if they aren't actually constructed
    %
    % Arguments:
    %     component_classnames (struct): Struct of component classnames
    %     components (struct): Struct of components or their classnames
    arguments
        component_classnames(1,1) struct
        components = component_classnames
    end

    % Generate an entry for each field which is of type rtd.util.mixins.Options
    fields = fieldnames(component_classnames).';
    componentoptions = struct;
    for fieldname = fields
        if ismember('rtd.util.mixins.Options', superclasses(components.(fieldname{1})))
            try
                componentoptions.(fieldname{1}) ...
                    = components.(fieldname{1}).getoptions();
            catch
                componentoptions.(fieldname{1}) ...
                    = feval([components.(fieldname{1}) '.defaultoptions']);
            end
        end
    end
end
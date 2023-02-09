classdef WorldEntity < matlab.mixin.Heterogeneous & rtd.util.mixins.Options & rtd.util.mixins.NamedClass & handle

    properties (Abstract)
        info rtd.entity.components.BaseInfoComponent
        state rtd.entity.components.BaseStateComponent
    end

    properties (Dependent)
        uuid
    end

    methods
        function uuid = get.uuid(self)
            uuid = self.info.uuid;
        end

        function update_name(self, name)
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
            option_struct = self.instanceOptions;
            if isfield(option_struct, 'component_options') ...
                    && isfield(option_struct.component_options, name)
                component = feval(option_struct.components.(name), varargin{:}, option_struct.component_options.(name));
            else
                component = feval(option_struct.components.(name), varargin{:});
            end
            self.(name) = component;
        end

        % TODO add inclusion, exclusion options
        function reset_components(self)
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
                self.(component_name{1}).reset(options.component_options.(component_name{1}));
            end
        end

        % Get all the options used for initialization of the robot
        function options = getoptions(self)
            % Update all the component options before returning.

            % Get a proposal set of options
            options = getoptions@rtd.util.mixins.Options(self);

            % Get all the component options
            options.component_options = rtd.sim.world.WorldEntity.get_componentoptions(options.components, self);

            % Merge it back into the stored options
            options = self.mergeoptions(options);
        end

    end

    methods (Static)

        function options = baseoptions()
            options.components = struct;
            options.component_options = struct;%get_componentoptions(components);
            options.component_logLevelOverride = [];
            options.verboseLevel = 'INFO';
            options.name = '';
        end

        function options = get_componentOverrideOptions(components)
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
            options.component_options = rtd.sim.world.WorldEntity.get_componentoptions(struct(component_handles{:}));
        end

        function componentoptions = get_componentoptions(component_classnames, components)
            arguments
                component_classnames
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
                            = eval([components.(fieldname{1}) '.defaultoptions()']);
                    end
                end
            end
        end
    end
end

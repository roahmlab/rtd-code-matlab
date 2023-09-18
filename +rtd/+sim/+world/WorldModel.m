classdef WorldModel < handle
    %Container for entities and systems in a given world for a simulation.
    
    properties
        all_entities(1,1) struct
        dynamic_entities(1,1) struct
        static_entities(1,1) struct
        systems(1,1) struct
    end
    methods (Static)
        function world = import(filename)
            % create the base world
            world = rtd.sim.world.WorldModel();
            world.mergeFromXML(filename)
        end
    end
    methods
        function mergeFromXML(self, filename)
            parser = matlab.io.xml.dom.Parser;
            dom = parser.parseFile(filename);
            rootNode = dom.getDocumentElement();
            self.mergeFromNode(rootNode);
        end
        function mergeFromNode(self, rootNode)
            static_nodelist = rootNode.getElementsByTagName('static_entities');
            dynamic_nodelist = rootNode.getElementsByTagName('dynamic_entities');
            system_nodelist = rootNode.getElementsByTagName('systems');
            for i=0:static_nodelist.getLength()-1
                element = static_nodelist.item(i);
                config_nodelist = element.getElementsByTagName('configuration_object');
                self.addEntityFromConfiguration(config_nodelist, 'static');
            end
            for i=0:dynamic_nodelist.getLength()-1
                element = dynamic_nodelist.item(i);
                config_nodelist = element.getElementsByTagName('configuration_object');
                self.addEntityFromConfiguration(config_nodelist, 'dynamic');
            end
            for i=0:system_nodelist.getLength()-1
                element = system_nodelist.item(i);
                config_nodelist = element.getElementsByTagName('configuration_object');
                self.addSystemFromConfiguration(config_nodelist);
            end
        end
        function addSystemFromConfiguration(self, config_nodelist)
            for i=0:config_nodelist.getLength()-1
                config_element = config_nodelist.item(i);
                [options_struct, ~, attributes] = rtd.functional.node2struct(config_element);
                % if we already have this configuration, just update it
                if isfield(self.systems, attributes.name) ...
                        && isa(self.systems.(attributes.name), attributes.class)
                    if ~isempty(fieldnames(options_struct))
                        self.systems.(attributes.name).reset(options=options_struct)
                    else
                        self.all_entities.(attributes.name).reset();
                    end
                % otherwise add it
                else
                    nvargs = namedargs2cell(options_struct);
                    system = feval(attributes.class, nvargs{:});
                    self.addSystem(system, attributes.name);
                end
            end
        end
        function addEntityFromConfiguration(self, config_nodelist, type)
            for i=0:config_nodelist.getLength()-1
                config_element = config_nodelist.item(i);
                [options_struct, ~, attributes] = rtd.functional.node2struct(config_element);
                % If we have it as the wrong type, error
                if isfield(self.all_entities, attributes.name) ...
                        && ~isfield(self.([type, '_entities']), attributes.name)
                    errMsg = MException('WorldModel:EntityUnexpectedType', ...
                        'Entity already exists as opposite type (dynamic/static)!');
                    throw(errMsg)
                end
                % if we already have this configuration, just update it
                if isfield(self.all_entities, attributes.name) ...
                        && isa(self.all_entities.(attributes.name), attributes.class) ...
                        && isequal(self.all_entities.(attributes.name).getoptions().components, options_struct.components)
                    self.all_entities.(attributes.name).reset(options=options_struct);
                % otherwise add/replace it
                else
                    nvargs = namedargs2cell(options_struct);
                    entity = feval(attributes.class, nvargs{:});
                    self.addEntity(entity, type=type, name=attributes.name);
                end
            end
        end
        % Add object
        function addEntity(self, entity, options)
            arguments
                self(1,1) rtd.sim.world.WorldModel
                entity(1,1) rtd.sim.world.WorldEntity
                options.type {mustBeMember(options.type,{'static','dynamic'})} = 'static'
                options.update_name(1,1) logical = false
                options.name {mustBeTextScalar} = ''
            end
            
            % Add the object to the world
            % Create a name for the object based on its classname if it
            % doesn't have a given name.
            if ~isempty(options.name)
                name = options.name;
            else
                name = entity.name;
            end
            if isempty(name)
                % Base classname
                name = entity.classname;
                % Get number to append by summing a regexp. It returns the
                % index of the occurance in each string, but since it's
                % limited to the first word anyway, it'll be 1 or empty.
                search_string = ['^', char(name), '\d+$'];
                all_names = fieldnames(self.all_entities);
                id = sum(cell2mat(regexp(all_names, search_string)));
                name = [char(name), num2str(id)];
            end
            if options.update_name
                entity.update_name(name);
            end
            self.all_entities.(name) = entity;
            
            % Save to the appropriate entity list
            if strcmp(options.type, 'dynamic')
                self.dynamic_entities.(name) = entity;
            else
                self.static_entities.(name) = entity;
            end
        end
        function addSystem(self, system, name, options)
            arguments
                self(1,1) rtd.sim.world.WorldModel
                system(1,1) rtd.sim.systems.SimulationSystem
                name {mustBeTextScalar}
                options.update_name(1,1) logical = false
                options.replace_system(1,1) logical = true
            end

            if isfield(self.systems, name)
                if ~options.replace_system
                    errMsg = MException('WorldModel:SystemExists', ...
                        'System name already exists!');
                    throw(errMsg)
                end
            end
            if options.update_name
                system.update_name(name);
            end
            self.systems.(name) = system;
        end
        function struct_out = getEntityComponent(self, component_name, options)
            arguments
                self rtd.sim.world.WorldModel   
                component_name {mustBeTextScalar}
                options.type {mustBeMember(options.type,{'all','static','dynamic'})} = 'all'
                options.MissingComponentAction {mustBeMember(options.MissingComponentAction,{'skip','empty','throw'})} = 'skip'
            end
            switch options.type
                case 'all'
                    struct_to_use = self.all_entities;
                case 'static'
                    struct_to_use = self.static_entities;
                case 'dynamic'
                    struct_to_use = self.dynamic_entities;
            end

            names = fieldnames(struct_to_use).';
            struct_out = struct;
            for name_cell=names
                name = name_cell{1};
                entity = struct_to_use.(name);
                % If the component isn't there, or isempty, execute the
                % missing component action
                if ~isprop(entity, component_name) || isempty(entity.(component_name))
                    switch options.MissingComponentAction
                        case 'skip'
                            continue
                        case 'empty'
                            component = [];
                        case 'throw'
                            errMsg = MException('WorldModel:MissingComponent', ...
                                'Component requests is either missing or empty!');
                            throw(errMsg)
                    end
                else
                    component = entity.(component_name);
                end
                struct_out.(name) = component;
            end
        end
        function rootNode = to_nodes(self, dom)
            % generate a dom for each
            rootNode = dom.createElement('world');
            if ~isempty(fieldnames(self.dynamic_entities))
                dynamic_node = dom.createElement('dynamic_entities');
                configuration_struct_to_dom_helper(dom, dynamic_node, self.dynamic_entities);
                rootNode.appendChild(dynamic_node);
            end
            if ~isempty(fieldnames(self.static_entities))
                static_node = dom.createElement('static_entities');
                configuration_struct_to_dom_helper(dom, static_node, self.static_entities);
                rootNode.appendChild(static_node);
            end
            if ~isempty(fieldnames(self.systems))
                system_node = dom.createElement('systems');
                configuration_struct_to_dom_helper(dom, system_node, self.systems);
                rootNode.appendChild(system_node);
            end
        end
        function export(self, filename)
            dom = matlab.io.xml.dom.Document('world');
            old_rootNode = dom.getDocumentElement();
            new_rootNode = self.to_nodes(dom);
            dom.replaceChild(new_rootNode, old_rootNode);
            writer = matlab.io.xml.dom.DOMWriter;
            writer.Configuration.FormatPrettyPrint = true;
            writer.writeToFile(dom, filename);
        end
    end
end

function configuration_struct_to_dom_helper(dom, rootNode, struct_to_use)
    names = fieldnames(struct_to_use).';
    for name_cell=names
        name = name_cell{1};
        struct_item = struct_to_use.(name);
        entity_classname = class(struct_item);
        attributes = struct;
        attributes.class = entity_classname;
        attributes.name = name;
        options_struct = struct;
        if ismember('rtd.util.mixins.Options', superclasses(struct_item))
            options_struct = struct_item.getoptions();
        end
        node = rtd.functional.struct2node(options_struct, dom, 'configuration_object', attributes);
        rootNode.appendChild(node);
    end
end

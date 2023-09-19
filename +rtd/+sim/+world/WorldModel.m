classdef WorldModel < handle
% Contains the entities and systems for a simulation.
%
% A WorldModel is a container for entities and systems. In any given
% simulation, there should be one WorldModel. The WorldModel is responsible
% for describing the entities and systems in the simulation, and the
% simulation itself is responsible for running the simulation.
%
% This class contains methods for importing and exporting the world to and
% from XML files.
%
% --- More Info ---
% Author: Adam Li (adamli@umich.edu)
% Written: 2023-09-18
%
% See also: rtd.sim.world.WorldEntity, rtd.sim.systems.SimulationSystem
%
% --- More Info ---
%

    properties
        % all_entities is a struct of all entities in the world
        all_entities(1,1) struct

        % dynamic_entities is a struct of all dynamic entities in the world.
        % it is a subset of all_entities
        dynamic_entities(1,1) struct

        % static_entities is a struct of all static entities in the world.
        % it is a subset of all_entities
        static_entities(1,1) struct

        % systems is a struct of all systems in the world
        systems(1,1) struct
    end
    
    % Entity and system management methods
    methods
        function addEntity(self, entity, options)
            % Add an entity to the world
            %
            % Arguments:
            %   entity(rtd.sim.world.WorldEntity): the entity to add
            %   options: Keyword arguments. See below.
            %
            % Keyword Arguments:
            %   type: the type of entity to add. Must be 'static' or
            %       'dynamic'. Default: 'static'
            %   update_name: whether to update the name of the entity to
            %       match the name given in the options. Default: false
            %   name: the name of the entity. If not given, the name will
            %       be the classname of the entity with a number appended
            %       to it. Default: ''
            %
            arguments
                self(1,1) rtd.sim.world.WorldModel
                entity(1,1) rtd.sim.world.WorldEntity
                options.type {mustBeMember(options.type,{'static','dynamic'})} = 'static'
                options.update_name(1,1) logical = false
                options.name {mustBeTextScalar} = ''
            end
            
            % Add the object to the world
            % Create a name for the object based on its classname if it
            % doesn't have a given name either passed in or as the entity itself.
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
            
            % Save to the appropriate entity struct
            if strcmp(options.type, 'dynamic')
                self.dynamic_entities.(name) = entity;
            else
                self.static_entities.(name) = entity;
            end
        end

        function addSystem(self, system, name, options)
            % Add a system to the world
            %
            % Arguments:
            %   system(rtd.sim.systems.SimulationSystem): the system to add
            %   name: the name of the system
            %   options: Keyword arguments. See below.
            %
            % Keyword Arguments:
            %   update_name: whether to update the name of the system to
            %       match the name given in the options. Default: false
            %   replace_system: whether to replace the system if it already
            %       exists. If false, an error will be thrown. Default: true
            %
            % Raises:
            %   SystemExists: if the system already exists and replace_system
            %       is false
            %
            arguments
                self(1,1) rtd.sim.world.WorldModel
                system(1,1) rtd.sim.systems.SimulationSystem
                name {mustBeTextScalar}
                options.update_name(1,1) logical = false
                options.replace_system(1,1) logical = true
            end

            % Check if the system already exists
            if isfield(self.systems, name)
                if ~options.replace_system
                    errMsg = MException('WorldModel:SystemExists', ...
                        'System name already exists!');
                    throw(errMsg)
                end
            end

            % Update the name if necessary and add to the world
            if options.update_name
                system.update_name(name);
            end
            self.systems.(name) = system;
        end

        function struct_out = getEntityComponent(self, component_name, options)
            % Get a component from all entities in the world
            %
            % Arguments:
            %   component_name: the name of the component to get
            %   options: Keyword arguments. See below.
            %
            % Keyword Arguments:
            %   type: the type of entity to get the component from. Must be
            %       'all', 'static', or 'dynamic'. Default: 'all'
            %   MissingComponentAction: what to do if the component is
            %       missing. Must be 'skip', 'empty', or 'throw'. If 'skip',
            %       the entity will be skipped. If 'empty', an empty
            %       component will be returned for that entity. If 'throw',
            %       an error will be thrown. Default: 'skip'
            %
            % Returns:
            %   struct_out: a struct of the components, with the same
            %       fields as the entities
            %
            % Raises:
            %   MissingComponent: if the component is missing and
            %       MissingComponentAction is 'throw'
            %
            arguments
                self(1,1) rtd.sim.world.WorldModel   
                component_name {mustBeTextScalar}
                options.type {mustBeMember(options.type,{'all','static','dynamic'})} = 'all'
                options.MissingComponentAction {mustBeMember(options.MissingComponentAction,{'skip','empty','throw'})} = 'skip'
            end

            % Determine which struct to use
            switch options.type
                case 'all'
                    struct_to_use = self.all_entities;
                case 'static'
                    struct_to_use = self.static_entities;
                case 'dynamic'
                    struct_to_use = self.dynamic_entities;
            end

            % Get the component from each entity and put it in a struct
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
    end

    % XML import/export static methods
    methods (Static)
        function world = import(filename)
            % Import a world from an XML file
            %
            % Arguments:
            %   filename: the filename to import from
            %
            % Returns:
            %   world: the world model
            %
            arguments
                filename {mustBeTextScalar}
            end

            % create a base worldmodel
            world = rtd.sim.world.WorldModel();
            world.mergeFromXML(filename)
        end
    end

    % XML import/export methods
    methods
        function mergeFromXML(self, filename)
            % Merge a world from an XML file
            %
            % Arguments:
            %   filename: the filename to import from
            %
            % Raises:
            %   EntityUnexpectedType: if an entity already exists as the
            %       opposite type (dynamic/static)
            %   InvalidVersion: if the version is not 0.1
            %
            arguments
                self(1,1) rtd.sim.world.WorldModel
                filename {mustBeTextScalar}
            end

            parser = matlab.io.xml.dom.Parser;
            dom = parser.parseFile(filename);
            rootNode = dom.getDocumentElement();
            self.mergeFromNode(rootNode);
        end

        function mergeFromNode(self, rootNode)
            % Merge a world from an XML node
            %
            % Arguments:
            %   rootNode: the node to import from
            %
            % Raises:
            %   EntityUnexpectedType: if an entity already exists as the
            %       opposite type (dynamic/static)
            %   InvalidVersion: if the version is not 0.1
            %
            arguments
                self(1,1) rtd.sim.world.WorldModel
                rootNode matlab.io.xml.dom.Node
            end

            % Verify that this is a version we can work with
            version = rootNode.getAttribute('format_version');
            if ~strcmp(version, '0.1')
                errMsg = MException('WorldModel:InvalidVersion', ...
                    'Invalid version!');
                throw(errMsg)
            end

            % Get the nodes and add them to the world
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

        function outstring = export(self, filename)
            % Export a world to an XML file
            %
            % Arguments:
            %   filename: the filename to export to. If empty, the XML
            %       string will be returned instead of written to a file.
            %       Default: ''
            %
            % Returns:
            %   outstring: the XML string if filename is empty
            %
            arguments
                self(1,1) rtd.sim.world.WorldModel
                filename {mustBeTextScalar} = ''
            end

            dom = matlab.io.xml.dom.Document('world');
            old_rootNode = dom.getDocumentElement();
            new_rootNode = self.to_nodes(dom);
            dom.replaceChild(new_rootNode, old_rootNode);
            writer = matlab.io.xml.dom.DOMWriter;
            writer.Configuration.FormatPrettyPrint = true;
            if ~isempty(filename)
                writer.writeToFile(dom, filename);
            else
                outstring = writer.writeToString(dom);
            end
        end
    end

    % Private helper methods for XML import/export
    methods (Access=private)
        function addSystemFromConfiguration(self, config_nodelist)
            % Add or update systems from a configuration node list
            %
            % Arguments:
            %   config_nodelist(matlab.io.xml.dom.NodeList): the node list
            %       to import systems from
            %
            arguments
                self(1,1) rtd.sim.world.WorldModel
                config_nodelist(1,1) matlab.io.xml.dom.NodeList
            end

            for i=0:config_nodelist.getLength()-1
                config_element = config_nodelist.item(i);
                [options_struct, ~, attributes] = rtd.functional.node2struct(config_element);
                % if we already have this system, just update it
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
            % Add or update entities from a configuration node list
            %
            % Arguments:
            %   config_nodelist(matlab.io.xml.dom.NodeList): the node list
            %       to import entities from
            %   type: the type of entity to add. Must be 'static' or
            %       'dynamic'
            %
            % Raises:
            %   EntityUnexpectedType: if an entity already exists as the
            %       opposite type (dynamic/static)
            %
            arguments
                self(1,1) rtd.sim.world.WorldModel
                config_nodelist(1,1) matlab.io.xml.dom.NodeList
                type {mustBeMember(type,{'static','dynamic'})}
            end

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

        function rootNode = to_nodes(self, dom)
            % Convert the world to XML nodes
            %
            % Arguments:
            %   dom(matlab.io.xml.dom.Document): the document to create the
            %       nodes in
            %
            % Returns:
            %   rootNode(matlab.io.xml.dom.Element): the root node of the
            %       world model
            %
            arguments
                self(1,1) rtd.sim.world.WorldModel
                dom(1,1) matlab.io.xml.dom.Document
            end
            
            % Create the root node, then add the entities and systems
            % if they exist
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
            % Add a version attribute for future later validation
            rootNode.setAttribute('format_version', '0.1');
        end
    end
end

function configuration_struct_to_dom_helper(dom, rootNode, struct_to_use)
    % Helper function to convert a struct to XML nodes
    %
    % Arguments:
    %   dom(matlab.io.xml.dom.Document): the document to create the
    %       nodes in
    %   rootNode(matlab.io.xml.dom.Node): the root node to add the
    %       nodes to
    %   struct_to_use(struct): the struct to 
    %
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

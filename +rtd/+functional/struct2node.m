function node = struct2node(struct_in, document, root_name, attributes_struct)
% Convert a struct to a DOM object
%
% This is a function that converts a struct to a DOM object. This is
% useful for writing out a struct to an XML file. The resulting DOM object
% can be conversely converted back to a struct using the node2struct
% function.
%
% Usage:
%   dom = matlab.io.xml.dom.Document('example_dom');
%   old_rootNode = dom.getDocumentElement();
%   new_rootNode = rtd.functional.struct2node(struct_in, dom, root_name, attributes_struct);
%   dom.replaceChild(new_rootNode, old_rootNode)
%   writer = matlab.io.xml.dom.DOMWriter;
%   writer.writeToFile(dom, 'example.xml');
%
% Arguments:
%   struct_in: The struct to convert to a DOM object
%   document: The document that DOM object is associated with
%   root_name: The name of the root node to store in the DOM object
%   attributes_struct: Attributes to associate with the root element
%
% Returns:
%   node: An element node created for the given document
%
% --- More Info ---
% Author: Adam Li (adamli@umich.edu)
% Written: 2023-09-14
% 
% See also: rtd.functional.node2struct
%
% --- More Info ---
%
    arguments
        struct_in(1,1) struct
        document(1,1) matlab.io.xml.dom.Document
        root_name {mustBeTextScalar} = 'root_node'
        attributes_struct(1,1) struct = struct('version', '0.1')
    end

%     doc = matlab.io.xml.dom.Document(root_name);
%     rootNode = doc.getDocumentElement();
    node = document.createElement(root_name);
    for name_cell=fieldnames(attributes_struct).'
        name = name_cell{1};
        node.setAttribute(name, attributes_struct.(name));
    end
    processStruct(document, node, struct_in);
%     dom = doc;
end

% --- Helper Functions ---

function rootNode = processStruct(document, rootNode, struct_to_parse)
% Process a struct into a DOM object node
%
% This is a function that converts a struct to a DOM object. This is
% useful for writing out a struct to an XML file. The resulting DOM object
% can be conversely converted back to a struct using the node2struct
% function.
%
% Arguments:
%   document: The document that DOM object is associated with
%   rootNode: The root node to append to
%   struct_to_parse: The struct to convert to a DOM object
%
% Returns:
%   rootNode: The root node with the struct appended
%
    arguments
        document(1,1) matlab.io.xml.dom.Document
        rootNode(1,1) matlab.io.xml.dom.Node
        struct_to_parse(1,1) struct
    end

    % Parse through the struct
    keys = fieldnames(struct_to_parse).';
    for key_cell=keys
        key = key_cell{1};
        val = struct_to_parse.(key);
        % Create our node
        node = createNode(document, key, val);
        if ~isempty(node)
            rootNode.appendChild(node);
        end
    end
end

function processDispatcher(document, node, val)
% Dispatches the processing of a value to the appropriate function
%
% This is a function that dispatches the processing of a value to the
% appropriate function. It serializes the value to a string and then
% assigns it to the node.
%
% Arguments:
%   document: The document that DOM object is associated with
%   node: The node to append to
%   val: The value to process
%
    arguments
        document(1,1) matlab.io.xml.dom.Document
        node(1,1) matlab.io.xml.dom.Node
        val
    end

    if isstruct(val)
        processStruct(document, node, val);

    elseif iscell(val)
        processCell(document, node, val);

    elseif ischar(val)
        node.setTextContent(val)

    elseif isnumeric(val) || islogical(val) || isstring(val)
        val = val(:);
        node.setTextContent(jsonencode(val, ConvertInfAndNaN=false));

    else
        error(['Encountered Unsupported Type ', class(val), '!'])
    end
end

function rootNode = processCell(document, rootNode, cells_to_parse)
% Process a cell into a DOM object node
%
% This is a function that converts a cell to a DOM object by converting
% each cell entry to a DOM object.
%
% Arguments:
%   document: The document that DOM object is associated with
%   rootNode: The root node to append to
%   cells_to_parse: The cell to convert to a DOM object
%
% Returns:
%   rootNode: The root node with the cell appended
%
    arguments
        document(1,1) matlab.io.xml.dom.Document
        rootNode(1,1) matlab.io.xml.dom.Node
        cells_to_parse cell
    end

    % Flatten the cells
    cells_to_parse = cells_to_parse(:).';
    for cell_entry=cells_to_parse
        val = cell_entry{1};
        % Create our node with an empty name
        node = createNode(document, '', val);
        if ~isempty(node)
            rootNode.appendChild(node);
        end
    end
end

function node = createNode(document, name, val)
% Create a node for a given value
%
% This is a function that creates a node for a given value. It adds the
% appropriate attributes to the node and then dispatches the processing of
% the value to the appropriate function.
%
% Arguments:
%   document: The document that DOM object is associated with
%   name: The name of the node to create
%   val: The value to process
%
% Returns:
%   node: The node created for the given value
%
    arguments
        document(1,1) matlab.io.xml.dom.Document
        name {mustBeTextScalar}
        val
    end

    % Create our node if we have a value
    if numel(val) == 0
        node = [];
        return
    end
    node = document.createElement(class(val));

    % Add a name attribute if we have one
    if ~isempty(name)
        node.setAttribute('name', name);
    end

    % Add the size attribute if we have a non-scalar value
    if numel(val) > 1
        node.setAttribute('size', jsonencode(size(val)));
    end
    
    processDispatcher(document, node, val);
end
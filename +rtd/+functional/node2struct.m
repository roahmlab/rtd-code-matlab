function [struct_out, root_name, attributes_struct] = node2struct(rootNode)
% Converts a DOM object to a struct
%
% This is a function that converts a DOM object to a struct. The struct
% will have the same structure as the XML file. This is useful for
% converting XML files to a struct for use in MATLAB, and is the
% counterpart to struct2node. 
%
% Usage:
%   parser = matlab.io.xml.dom.Parser;
%   dom = parser.parseFile('example.xml');
%   rootNode = dom.getDocumentElement();
%   [struct_out, root_name, attributes_struct] = rtd.functional.node2struct(rootNode)
%
% Arguments
%   node - A DOM node object
%
% Returns:
%   struct_out - A struct with the same structure as the XML file's node
%   root_name - The name of the root element
%   attributes_struct - Attributes associated with the root element
%
% --- More Info ---
% Author: Adam Li (adamli@umich.edu)
% Written: 2023-09-14
%
% See Also: rtd.functional.struct2node
%
% --- More Info ---
%
    arguments
        rootNode(1,1) matlab.io.xml.dom.Node
    end

%     rootNode = dom.getDocumentElement();
    root_name = rootNode.TagName;
    attributes_struct = struct;
    for i=0:rootNode.getAttributes().getLength()-1
        attr_node = rootNode.getAttributes().item(i);
        attributes_struct.(attr_node.Name) = attr_node.TextContent;
    end
    struct_out = parseStruct(rootNode);
end

% --- Helper Functions ---

function struct_out = parseStruct(element)
% Parses a struct from an element
%
% This is a helper function that parses a struct from an element. It
% recursively calls itself to parse sub-structs.
%
% Arguments:
%   element - A DOM element object
%
% Returns:
%   struct_out - A struct with the same structure as the XML file's node
%
    arguments
        element(1,1) matlab.io.xml.dom.Node
    end

    struct_out = struct;
    for child=element.Children
        if isa(child, 'matlab.io.xml.dom.Text')
            % We don't expect any plain text nodes here.
            % AKA ignore whitespace
            continue
        end
        [key, value] = parseElement(child);
        struct_out.(key) = value;
    end
end

function cells_out = parseCells(element, size_vec)
% Parses a cell array from an element
%
% This is a helper function that parses a cell array from an element. It
% recursively calls itself to parse sub-cells.
%
% Arguments:
%   element - A DOM element object
%   size_vec - The size of the cell array
%
% Returns:
%   cells_out - A cell array with the same structure as the XML file's node
%
    arguments
        element(1,1) matlab.io.xml.dom.Node
        size_vec(1,:) double
    end

    cells_out = cell(prod(size_vec),1);
    idx = 1;
    for child=element.Children
        if isa(child, 'matlab.io.xml.dom.Text')
            % We don't expect any plain text nodes here.
            % AKA ignore whitespace
            continue
        end
        [~, subvalue] = parseElement(child);
        cells_out{idx} = subvalue;
        idx = idx + 1;
    end
    cells_out = reshape(cells_out, size_vec);
end

function [key, value] = parseElement(element)
% Parses an element
%
% This is a helper function that parses an element. It determines the type
% of the element and calls the appropriate function to parse it.
%
% Arguments:
%   element - A DOM element object
%
% Returns:
%   key - The name of the element
%   value - The value of the element
%
    arguments
        element(1,1) matlab.io.xml.dom.Node
    end

    % Get the type of the element
    classtype = element.TagName;
    key = element.getAttribute('name');
    size_vec = element.getAttribute('size');
    % If the size is empty, then it's a scalar
    if isempty(size_vec)
        size_vec = [1,1];
    else
        size_vec = jsondecode(size_vec).';
    end
    
    % Get the empty type for dispatching, also get the type handle
    emptytype = feval([classtype, '.empty']);
    typehandle = str2func(classtype);

    % Parse the element
    if isstruct(emptytype)
        value = parseStruct(element);

    elseif iscell(emptytype)
        value = parseCells(element, size_vec);

    elseif ischar(emptytype)
        value = element.TextContent;

    elseif isstring(emptytype)
        value = jsondecode(element.TextContent);
        value = string(value);
        value = reshape(value, size_vec);

    elseif isnumeric(emptytype) || islogical(emptytype)
        % Decode and reshape the value
        value = jsondecode(element.TextContent);
        value = reshape(value, size_vec);
        % Convert the type
        value = typehandle(value);
        
    else
        error(['Encountered Unsupported Type ', classtype, '!'])
    end
end
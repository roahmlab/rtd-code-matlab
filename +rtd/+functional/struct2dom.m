function dom = struct2dom(struct_in, root_name, version_string)
% Convert a struct to a DOM object
%
% This is a function that converts a struct to a DOM object. This is
% useful for writing out a struct to an XML file. The resulting DOM object
% can be conversely converted back to a struct using the dom2struct
% function.
%
% Usage:
%   dom = struct2dom(struct_in, root_name, version_string)
%   writer = matlab.io.xml.dom.DOMWriter;
%   writer.writeToFile(dom, 'example.xml');
%
% Arguments:
%   struct_in: The struct to convert to a DOM object
%   root_name: The name of the root node to store in the DOM object
%   version_string: A version string to save in the DOM object
%
% Returns:
%   dom: The DOM object
%
% --- More Info ---
% Author: Adam Li (adamli@umich.edu)
% Written: 2023-09-14
% 
% See also: rtd.functional.dom2struct
%
% --- More Info ---
%
    arguments
        struct_in(1,1) struct
        root_name(1,:) char = 'root_node'
        version_string(1,:) char = '0.1'
    end

    doc = matlab.io.xml.dom.Document(root_name);
    rootNode = doc.getDocumentElement();
    rootNode.setAttribute('version', version_string);
    processStruct(doc, rootNode, struct_in);
    dom = doc;
end

% TODO Document helpers
function rootNode = processStruct(document, rootNode, struct_to_parse)

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

% Flatten the cells
cells_to_parse = cells_to_parse(:).';
for cell_entry=cells_to_parse
    val = cell_entry{1};
    % Create our node
    node = createNode(document, [], val);
    if ~isempty(node)
        rootNode.appendChild(node);
    end
end
end

function node = createNode(document, name, val)
    % Create our node
    if numel(val) == 0
        node = [];
        return
    end
    node = createElement(document, class(val));
    if ~isempty(name)
        node.setAttribute('name', name);
    end
    if numel(val) > 1
        node.setAttribute('size', jsonencode(size(val)));
    end
    processDispatcher(document, node, val);
end
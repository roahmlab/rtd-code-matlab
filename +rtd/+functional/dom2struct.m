function [struct_out, root_name, version] = dom2struct(dom)
% Converts a DOM object to a struct
%
% This is a function that converts a DOM object to a struct. The struct
% will have the same structure as the XML file. This is useful for
% converting XML files to a struct for use in MATLAB, and is the
% counterpart to struct2dom. 
%
% Usage:
%   parser = matlab.io.xml.dom.Parser;
%   dom = parser.parseFile('example.xml');
%   [struct_out, root_name, version] = dom2struct(dom)
%
% Arguments
%   dom - A DOM object
%
% Returns:
%   struct_out - A struct with the same structure as the XML file
%   root_name - The name of the root element
%   version - The version of the XML file (which would have been provided to struct2dom)
%
% --- More Info ---
% Author: Adam Li (adamli@umich.edu)
% Written: 2023-09-14
%
% See Also: rtd.functional.struct2dom
%
% --- More Info ---
%
    arguments
        dom(1,1) matlab.io.xml.dom.Document
    end

    rootNode = dom.getDocumentElement();
    root_name = rootNode.TagName;
    version = rootNode.getAttribute('version');
    struct_out = parseStruct(dom.getDocumentElement());
end

% TODO Document helpers
function struct_out = parseStruct(element)
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
    classtype = element.TagName;
    key = element.getAttribute('name');
    size_vec = element.getAttribute('size');
    if isempty(size_vec)
        size_vec = [1,1];
    else
        size_vec = jsondecode(size_vec).';
    end
    
    emptytype = feval([classtype, '.empty']);
    typehandle = str2func(classtype);

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
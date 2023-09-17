function encodedout = htmlencode(in_string)
% HTMLENCODE Encode string for HTML display
%
%   ENCODEDOUT = HTMLENCODE(IN_STRING) encodes the string IN_STRING for
%   HTML display. The output ENCODEDOUT is a string that can be displayed
%   in a browser.
%
%   Example:
%       htmlencode('<a href="http://www.mathworks.com">MATLAB</a>')
%
%   See also rtd.functional.htmldecode
%
    arguments
        in_string {mustBeTextScalar}
    end

    dummydoc = matlab.io.xml.dom.Document('dummydoc');
    textNode = createTextNode(dummydoc, in_string);
    writer = matlab.io.xml.dom.DOMWriter;
    encodedout = writer.writeToString(textNode);
end

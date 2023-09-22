function decodedout = htmldecode(in_string)
%HTMLDECODE Decode HTML entities
%
%   DECODEDOUT = HTMLDECODE(IN_STRING) decodes the HTML entities in
%   IN_STRING and returns the decoded text in DECODEDOUT.
%
%   Example:
%       htmldecode('AT&amp;T')
%
%   See also rtd.functional.htmlencode
%
    arguments
        in_string {mustBeTextScalar}
    end

    parser = matlab.io.xml.dom.Parser;
    encoded = ['<a>',char(in_string),'</a>'];
    out = parser.parseString(encoded);
    decodedout = out.Children.TextContent;
end

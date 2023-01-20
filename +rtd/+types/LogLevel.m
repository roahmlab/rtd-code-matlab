classdef LogLevel < uint8
% Enumeration type for logging levels.
%
% Note:
%     Log levels `GENERAL` and `OFF` have the same level so that `GENERAL` 
%     output will always show.
%     These levels are roughly selected based on log4j and
%     https://www.section.io/engineering-education/how-to-choose-levels-of-logging/.
%
% --- More Info ---
% Author: Adam Li (adamli@umich.edu)
% Written: 2022-12-09
%
% See also rtd.mixins.NamedClass
%
% --- More Info ---
%
    enumeration
        ALL       (0)
        TRACE     (4)
        DEBUG     (8)
        INFO      (12)
        WARN      (16)
        ERROR     (20)
        GENERAL   (24)
        OFF       (24)
    end
end
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
% Last Revised: 2023-01-20 (Adam Li)
%
% See also rtd.util.mixins.NamedClass
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
    
    % Add validation for the comparison operators so we can compare to the
    % string equivalents
    methods
        function tf = lt(self, other)
            arguments
                self rtd.util.types.LogLevel
                other rtd.util.types.LogLevel
            end
            tf = lt@uint8(self, other);
        end
        function tf = gt(self, other)
            arguments
                self rtd.util.types.LogLevel
                other rtd.util.types.LogLevel
            end
            tf = gt@uint8(self, other);
        end
        function tf = le(self, other)
            arguments
                self rtd.util.types.LogLevel
                other rtd.util.types.LogLevel
            end
            tf = le@uint8(self, other);
        end
        function tf = ge(self, other)
            arguments
                self rtd.util.types.LogLevel
                other rtd.util.types.LogLevel
            end
            tf = ge@uint8(self, other);
        end
        function tf = ne(self, other)
            arguments
                self rtd.util.types.LogLevel
                other rtd.util.types.LogLevel
            end
            tf = ne@uint8(self, other);
        end
        function tf = eq(self, other)
            arguments
                self rtd.util.types.LogLevel
                other rtd.util.types.LogLevel
            end
            tf = eq@uint8(self, other);
        end
    end
end
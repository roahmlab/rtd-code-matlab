classdef LogLevel < uint8
    enumeration
        % https://www.section.io/engineering-education/how-to-choose-levels-of-logging/
        ALL       (0)
        TRACE     (4)
        DEBUG     (8)
        INFO      (12)
        WARN      (16)
        ERROR     (20)
        % Give the below two the same so general globally applies
        GENERAL   (24)
        OFF       (24)
    end
end
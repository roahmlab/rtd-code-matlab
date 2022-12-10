classdef EntityInfo < handle
    % EntityInfo This might go later
    properties (Abstract)
        dimension uint8 {mustBeMember(dimension,[2, 3])}
    end
    methods (Abstract)
        reset(self)
    end
end
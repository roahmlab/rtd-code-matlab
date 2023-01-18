classdef EntityInfo < mixins.UUIDbase & handle
    % EntityInfo This might go later
    properties (Abstract)
        dimension double {mustBeMember(dimension,[2, 3])}
    end
    methods (Abstract)
        reset(self)
    end
end
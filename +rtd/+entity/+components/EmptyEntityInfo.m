classdef EmptyEntityInfo < rtd.entity.components.BaseInfoComponent & handle
    % EntityInfo This might go later
    properties
        dimension = 2
    end
    methods
        function reset(self)
            % NOP
        end
    end
end
classdef Group < matlab.mixin.Heterogeneous & handle
    % Make it so we can initialize it
    methods (Sealed, Access = protected)
        function load_all(self)
            for group=self(:).'
                group.load()
            end
        end
    end
    methods (Static, Sealed, Access = protected)
        function default_object = getDefaultScalarElement
            default_object = loaders.RSLoader.GroupLoader;
        end
    end
end
classdef PatchVisualObject < matlab.mixin.Heterogeneous & handle
    properties (Abstract)
        plot_data (1,1) struct
    end
    methods (Abstract)
        plot(self,options)
    end
    % Make it so we can initialize it
    methods (Static, Sealed, Access = protected)
        function default_object = getDefaultScalarElement
            default_object = rtd.core.systems.patch_visual.EmptyPatchVisualObject;
        end
    end
end
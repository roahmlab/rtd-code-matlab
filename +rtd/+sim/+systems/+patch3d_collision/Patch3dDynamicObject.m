classdef Patch3dDynamicObject < matlab.mixin.Heterogeneous & handle
    methods (Abstract)
        % Get the Patch3dObject for whatever options.
        % Must have an Name-Value pair to specify a time.
        % should return a Patch3dObject!
        out = getCollisionObject(self,options)
    end
    % Make it so we can initialize it
    methods (Static, Sealed, Access = protected)
        function default_object = getDefaultScalarElement
            default_object = rtd.sim.systems.patch3d_collision.EmptyPatch3dDynamicObject;
        end
    end
end
classdef EmptyPatch3dDynamicObject < rtd.core.systems.patch3d_collision.Patch3dDynamicObject & handle
    methods
        % Get the Patch3dObject for whatever options.
        % Must have an Name-Value pair to specify a time.
        % should return a Patch3dObject!
        function out = getCollisionObject(self,options)
            arguments
                self
                options.time = 0
            end
            out = rtd.core.systems.patch3d_collision.Patch3dObject;
        end
    end
end
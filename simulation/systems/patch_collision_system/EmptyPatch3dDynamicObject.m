classdef EmptyPatch3dDynamicObject < Patch3dDynamicObject & handle
    methods
        % Get the Patch3dObject for whatever options.
        % Must have an Name-Value pair to specify a time.
        % should return a Patch3dObject!
        function out = get_patch3dObject(self,options)
            arguments
                self
                options.time = 0
            end
            out = Patch3dObject;
        end
    end
end
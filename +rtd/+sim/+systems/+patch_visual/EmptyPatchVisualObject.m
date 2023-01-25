classdef EmptyPatchVisualObject < rtd.sim.systems.patch_visual.PatchVisualObject & handle
    properties
        plot_data = struct.empty()
    end
    methods
        % Get the Patch3dObject for whatever options.
        % Must have an Name-Value pair to specify a time.
        % should return a Patch3dObject!
        function plot(self,options)
            arguments
                self
                options.time = 0
            end
            % NOP
        end
    end
end
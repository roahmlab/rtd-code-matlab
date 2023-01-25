classdef Patch3dObject < rtd.util.mixins.UUID
    properties
        parent_uuid
        faces(:,:) uint32
        vertices(:,3) double
        group_ends(1,:) uint32 % the end of vertex groups
        centers(:,3) double
    end
    methods
        function self = Patch3dObject(parent_uuid, faces, vertices, group_ends, centers)
            arguments
                parent_uuid = []
                faces = []
                vertices = []
                group_ends = size(vertices,1)
                centers = []
            end
            self.parent_uuid = parent_uuid;
            self.faces = faces;
            self.vertices = vertices;
            if all(group_ends > 0)
                self.group_ends = group_ends;
            end
            self.centers = centers;
        end
        
        function [collision, pair] = inCollision(self, other)
            arguments
                self rtd.sim.systems.patch3d_collision.Patch3dObject
                other rtd.sim.systems.patch3d_collision.Patch3dObject
            end
            % Don't check collision if they share the same parent_uuid
            if strcmp(self.parent_uuid, other.parent_uuid)
                collision = false;
                pair = [];
                return
            end
            % Create and check AABB for quick check
            [min1, max1] = bounds(self.vertices);
            [min2, max2] = bounds(other.vertices);
            
            % If they don't intersect, skip
            if any(min1 > max2) || any(min2 > max1)
                collision = false;
                pair = [];
                return
            end
                
            % Create struct for compat
            surf1 = struct('faces', self.faces, 'vertices', self.vertices);
            surf2 = struct('faces', other.faces, 'vertices', other.vertices);
            check = SurfaceIntersection(surf1, surf2);
            collision = full(any(check(:)));
            % add this as compat
            % Check centers if self or other has centers
            if ~isempty(self.centers)
                collision = collision || check_centers(self, other);
            end
            if ~isempty(other.centers)
                collision = collision || check_centers(other, self);
            end 
            pair = [];
            if collision
                pair = {self.parent_uuid, other.parent_uuid};
            end
        end
    end
end

% Introduced for compat to retain parity with original
function out = check_centers(patch3dObject_centers, patch3dObject_check)
    arguments
        patch3dObject_centers rtd.sim.systems.patch3d_collision.Patch3dObject
        patch3dObject_check rtd.sim.systems.patch3d_collision.Patch3dObject
    end
    out = false;
    start = 1;
    % iterate over all vertex groups
    for end_idx = patch3dObject_check.group_ends
        out = out || ...
            any(inhull(...
                patch3dObject_centers.centers, ...
                patch3dObject_check.vertices(start:end_idx,:) ...
            ));
        start = end_idx + 1;
    end
end
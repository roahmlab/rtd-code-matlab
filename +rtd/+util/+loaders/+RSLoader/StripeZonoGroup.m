classdef StripeZonoGroup < rtd.util.loaders.RSLoader.Group & handle
    properties (Access=protected)
        zonos
    end
    properties
        parent
        stripe_idx
    end
    properties (Dependent)
        num_sets
    end

    methods
        function self = StripeZonoGroup(parent, stripe_idx)
            self.parent = parent;
            self.stripe_idx = stripe_idx;
            
            if parent.loader.preload_meta
                self.getZonos();
            end
        end
        
        function zonos = getZonos(self, idx)
            arguments
                self rtd.util.loaders.RSLoader.StripeZonoGroup
                idx (:,1) double = []
            end
            
            if isempty(self.zonos)
                all_groups = self.parent.getGroup();
                self.zonos = arrayfun(@(g)g.getZonos(self.stripe_idx), all_groups);
            end
            
            if isempty(idx)
                % Get all zonos if empty
                zonos = self.zonos;
            else
                % Otherwise expand the idx's so we can get start and ends
                idx = idx + 1;
                zonos = self.zonos(idx);
            end
        end
    end

    methods
        function num_sets = get.num_sets(self)
            num_sets = self.parent.num_groups;
        end
    end
end
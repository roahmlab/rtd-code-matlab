classdef GroupLoader < rtd.util.loaders.RSLoader.Group & handle
    properties
        loader
        parent
        indices
    end
    methods
        function group = construct(self)
            arguments
                self (1,1) rtd.util.loaders.RSLoader.GroupLoader
            end
            
            if isempty(self.loader) || isempty(self.parent) || isempty(self.indices)
                error('GroupLoader missing required data to construct')
            end
            
            % If we have a collection group
            if self.indices(1) > 0
                info = self.parent.info.Groups(self.indices(1));
                group = rtd.util.loaders.RSLoader.CollectionGroup(self.loader, info);
                return
            end
            
            % Otherwise we have a terminal set group, so fine set type
            info = self.parent.info.Datasets(-self.indices(1));
            type_id = find(matches({info.Attributes.Name}, 'set_type'));
            if ~isempty(type_id)
                type_id = info.Attributes(type_id).Value;
            else
                type_id = 0;
            end
            
            switch type_id
                % If we have a zonotope terminal group
                case 0
                    group = rtd.util.loaders.RSLoader.ZonoGroup(self.loader, self.parent, -self.indices(1));

                % If we have the polynomial zonotope terminal group
                case 1
                    group = rtd.util.loaders.RSLoader.PolyZonoGroup(self.loader, self.parent, -self.indices);

                % Unknown set type.
                otherwise
                    parent_path = self.parent.group_path;
                    dataset_path = [parent_path, '/', info.Name];
                    error(['Unknown set type encountered in file for dataset: ', dataset_path]);
            end
        end
    end
end
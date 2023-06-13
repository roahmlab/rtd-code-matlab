classdef ZonoGroup < loaders.RSLoader.Group & handle

    properties
        attributes
    end
    properties (Dependent)
        group_path
        num_sets
    end

    properties (Access=protected)
        loader
        parent
        dataset_idx

        % TODO
        rawdata_cache
        zono_cache
    end

    properties (Dependent, Access=protected)
        info
        rawdata_size
    end

    methods
        function self = ZonoGroup(loader, parent, dataset_idx)
            arguments
                loader (1,1)
                parent (1,1)
                dataset_idx (1,1) uint32
            end
            self.loader = loader;
            self.parent = parent;
            self.dataset_idx = dataset_idx;
            
            if loader.preload_meta
                self.load();
            end
        end
        
        function load(self)
            % Some preprocessing
            self.validateStoreAttributes();
        end
        
        function zonos = getZonos(self, idx)
            arguments
                self loaders.RSLoader.ZonoGroup
                idx (:,1) double = []
            end
            
            if isempty(idx)
                % Get all zonos if empty
                starts = self.attributes.indices(1:end-1);
                ends = self.attributes.indices(2:end);
                num = self.attributes.num_sets;
            else
                % Otherwise expand the idx's so we can get start and ends
                starts = idx;
                ends = starts + 1;
                num = length(starts);
                
                starts = self.attributes.indices(starts + 1);
                ends = self.attributes.indices(ends + 1);
            end
            
            % Get the rawdata and create the zonotopes
            [lower, upper] = bounds([starts; ends]);
            rawdata = self.get_data([lower upper]);
            % Create the zonotopes (cells for now)
            zonos = arrayfun( ...
                        @(i)zonotope(rawdata(:,starts(i)-lower+1:ends(i)-lower)), ...
                        1:num, ...
                        UniformOutput=false);
        end
    end
    
    methods (Access=protected)
        function validateStoreAttributes(self)
            % Get the proposed attributes
            pa = cell2struct({self.info.Attributes.Value},{self.info.Attributes.Name},2);
            % Validate that this is a collection group
            % Build the default attributes
            da.set_type = 0;
            da.indices = [];
            da.num_sets = 0;

            % Merge in
            fields = fieldnames(pa).';
            for fieldname = fields
                da.(fieldname{1}) = pa.(fieldname{1});
            end

            % Make sure they make sense
            if length(da.indices) - 1 ~= da.num_sets
                error(['Malformed Zonotope, unexpected indices or num_sets provided found for ', self.group_path])
            end

            % Store the attributes
            self.attributes = da;
        end
        
        function rawdata = get_data(self, range)
            arguments
                self loaders.RSLoader.ZonoGroup
                range (1,2) double = [self.attributes.indices(1) self.attributes.indices(end)]
            end
            
            % TODO caching
            
            start_idx = [1, range(1)+1];
            end_idx = [self.rawdata_size(1), range(2)];
            
            rawdata = self.loader.get_data(self.group_path, start_idx, end_idx);
        end
    end

    methods
        function info = get.info(self)
            info = self.parent.info.Datasets(self.dataset_idx);
        end

        function group_path = get.group_path(self)
            group_path = [self.parent.group_path, '/', self.info.Name];
        end

        function num_sets = get.num_sets(self)
            num_sets = self.attributes.num_sets;
        end

        function rawdata_size = get.rawdata_size(self)
            rawdata_size = self.info.Dataspace.Size;
        end
    end
end
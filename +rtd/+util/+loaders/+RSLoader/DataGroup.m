classdef DataGroup < rtd.util.loaders.RSLoader.Group & handle

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
        matdata_cache
        not_cached
    end

    properties (Dependent, Access=protected)
        info
        rawdata_size
    end

    methods
        function self = DataGroup(loader, parent, dataset_idx)
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
            if loader.preload_data
                self.getMatData();
            end
        end
        
        function load(self)
            % Some preprocessing
            self.validateStoreAttributes();
            
            % Prepare the cache
            if self.loader.do_caching
                self.not_cached = true(1, self.num_sets);
                self.matdata_cache = cell(1, self.num_sets);
            end
        end
        
        function matdata = getMatData(self, idx)
            arguments
                self rtd.util.loaders.RSLoader.DataGroup
                idx (:,1) double = (1:self.attributes.num_sets)-1
            end
            
            idx = idx + 1;
            if self.loader.do_caching
                % Get the zonotopes that we want to cache
                uncached_mask = self.not_cached(idx);
                to_cache = idx(uncached_mask);
                if ~isempty(to_cache)
                    self.matdata_cache(to_cache) = self.getRawMatData(to_cache);
                    self.not_cached(to_cache) = false;
                end
                % Create the return
                matdata = self.matdata_cache(idx);
            else
                % Just get the raw zonos everytime
                matdata = self.getRawMatData(idx);
            end
        end
    end
    
    methods (Access=protected)
        function matdata = getRawMatData(self, idx)
            % Helper function to get zonotopes, 1-indexed to match matlab
            % Otherwise expand the idx's so we can get start and ends
            starts = idx;
            ends = starts + 1;
            num = length(starts);

            starts = self.attributes.indices(starts);
            ends = self.attributes.indices(ends);

            % Get the rawdata and create the zonotopes
            [lower, upper] = bounds([starts; ends]);
            rawdata = self.get_data([lower upper]);
            % Create the flat expmat as cells
            matdata = arrayfun( ...
                        @(i)rawdata(:,starts(i)-lower+1:ends(i)-lower), ...
                        1:num, ...
                        UniformOutput=false);
        end
        
        function validateStoreAttributes(self)
            % Get the proposed attributes
            pa = cell2struct({self.info.Attributes.Value},{self.info.Attributes.Name},2);
            % Validate that this is a collection group
            % Build the default attributes
            da.indices = [];
            da.num_sets = 0;

            % Merge in
            fields = fieldnames(pa).';
            for fieldname = fields
                da.(fieldname{1}) = pa.(fieldname{1});
            end

            % Make sure they make sense
            if length(da.indices) - 1 ~= da.num_sets
                error(['Malformed Data, unexpected indices or num_sets provided found for ', self.group_path])
            end

            % Store the attributes
            self.attributes = da;
        end
        
        function rawdata = get_data(self, range)
            arguments
                self rtd.util.loaders.RSLoader.DataGroup
                range (1,2) double = [self.attributes.indices(1) self.attributes.indices(end)]
            end
            
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
classdef PolyZonoGroup < rtd.util.loaders.RSLoader.Group & handle

    properties
        attributes
    end
    properties (Dependent)
        group_path
        num_sets
        g_size
    end

    properties (Access=protected)
        loader
        parent
        dataset_idxs

        % TODO
        zono_cache
        not_cached

        % extra groups
        exp_group
        id_group
    end

    properties (Dependent, Access=protected)
        info
        rawdata_size
    end

    methods
        function self = PolyZonoGroup(loader, parent, dataset_idxs)
            arguments
                loader (1,1)
                parent (1,1)
                dataset_idxs (1,:) uint32
            end
            self.loader = loader;
            self.parent = parent;
            self.dataset_idxs = dataset_idxs;
            
            if loader.preload_meta
                self.load();
            end
            if loader.preload_data
                self.getZonos();
            end
        end
        
        function load(self)
            % Some preprocessing
            self.validateStoreAttributes();
            
            % If we have an exp group or an id group, load them too
            if ~isempty(self.attributes.exp_id)
                self.exp_group = rtd.util.loaders.RSLoader.DataGroup(self.loader, self.parent, self.dataset_idxs(self.attributes.exp_id));
                self.exp_group.load();
                if self.loader.preload_data
                    self.exp_group.getMatData();
                end
                if self.exp_group.num_sets ~= self.num_sets
                    error('Provided Exp Group for the PolyZono appears to have the wrong number of sets!')
                end
            end
            if ~isempty(self.attributes.id_id)
                self.id_group = rtd.util.loaders.RSLoader.DataGroup(self.loader, self.parent, self.dataset_idxs(self.attributes.id_id));
                self.id_group.load();
                if self.loader.preload_data
                    self.id_group.getMatData();
                end
                if self.id_group.num_sets ~= self.num_sets
                    error('Provided ID Group for the PolyZono appears to have the wrong number of sets!')
                end
            end

            % Prepare the cache
            if self.loader.do_caching
                self.not_cached = true(1, self.num_sets);
                self.zono_cache = cell(1, self.num_sets);
            end
        end
        
        function zonos = getZonos(self, idx)
            arguments
                self rtd.util.loaders.RSLoader.PolyZonoGroup
                idx (:,1) double = (1:self.attributes.num_sets)-1
            end
            
            idx = idx + 1;
            if self.loader.do_caching
                % Get the zonotopes that we want to cache
                uncached_mask = self.not_cached(idx);
                to_cache = idx(uncached_mask);
                if ~isempty(to_cache)
                    self.zono_cache(to_cache) = self.getRawZonos(to_cache);
                    self.not_cached(to_cache) = false;
                end
                % Create the return
                zonos = self.zono_cache(idx);
            else
                % Just get the raw zonos everytime
                zonos = self.getRawZonos(idx);
            end
        end
    end
    
    methods (Access=protected)
        function zonos = getRawZonos(self, idx)
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
            % Get the data as cells of matrices
            zonos = arrayfun( ...
                        @(i)rawdata(:,starts(i)-lower+1:ends(i)-lower), ...
                        1:num, ...
                        UniformOutput=false);

            if ~isempty(self.id_group)
                id_data = self.id_group.getMatData(idx-1);
            else
                id_data = arrayfun(@(num)(0:num-1).', self.g_size(idx), UniformOutput=false);
            end
            if ~isempty(self.exp_group)
                exp_mat = self.exp_group.getMatData(idx-1);
                for i=1:num
                    n_id = id_data{i};
                    n_g = self.g_size(i);
                    exp_mat{i} = reshape(exp_mat{i}, n_id, n_g);
                end
            else
                exp_mat = cell(1, num);
                for i=1:num
                    n_id = id_data{i};
                    n_g = self.g_size(i);
                    if n_id ~= n_g
                        error(['ExpMat appears to be missing for ', num2str(i-1)]);
                    end
                    exp_mat{i} = eye(n_id);
                end
            end

            % convert to the polynomial zonotopes
            for i=1:num
                Z = zonos{i};
                c = Z(:,1);
                G = Z(:,2:1+self.g_size(i));
                Grest = Z(:,2+self.g_size(i):end);
                zonos{i} = armour.pz_roahm.polyZonotope_ROAHM(c, G, Grest, exp_mat{i}, id_data{i});
            end
        end
        
        function validateStoreAttributes(self)
            % Get the proposed attributes
            pa = cell2struct({self.info.Attributes.Value},{self.info.Attributes.Name},2);
            % Validate that this is a collection group
            % Build the default attributes
            da.set_type = 1;
            da.indices = [];
            da.num_sets = 0;
            da.g_size = [];
            da.exp_id = [];
            da.id_id = [];

            % Merge in
            fields = fieldnames(pa).';
            for fieldname = fields
                da.(fieldname{1}) = pa.(fieldname{1});
            end

            % Make sure they make sense
            if length(da.indices) - 1 ~= da.num_sets
                error(['Malformed Poly Zonotope, unexpected indices or num_sets provided found for ', self.group_path])
            end

            % Store the attributes
            self.attributes = da;
        end
        
        function rawdata = get_data(self, range)
            arguments
                self rtd.util.loaders.RSLoader.PolyZonoGroup
                range (1,2) double = [self.attributes.indices(1) self.attributes.indices(end)]
            end
            
            start_idx = [1, range(1)+1];
            end_idx = [self.rawdata_size(1), range(2)];
            rawdata = self.loader.get_data(self.group_path, start_idx, end_idx);
        end
    end

    methods
        function info = get.info(self)
            info = self.parent.info.Datasets(self.dataset_idxs(1));
        end

        function group_path = get.group_path(self)
            group_path = [self.parent.group_path, '/', self.info.Name];
        end

        function num_sets = get.num_sets(self)
            num_sets = self.attributes.num_sets;
        end

        function g_size = get.g_size(self)
            g_size = self.attributes.g_size;
        end

        function rawdata_size = get.rawdata_size(self)
            rawdata_size = self.info.Dataspace.Size;
        end
    end
end
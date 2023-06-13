classdef CollectionGroup < loaders.RSLoader.Group & handle
    properties (Access=?loaders.RSLoader.Group)
        loader
        info
        
        prefix
        
        group_lookup_table
        unloaded_groups
        groups
        search_set
    end
    properties (Dependent)
        group_path
        num_groups
    end
    properties
        attributes
    end
    methods
        function self = CollectionGroup(loader, info)
            arguments
                loader (1,1)
                info (1,1) struct
            end
            self.loader = loader;
            self.info = info;
            
            if loader.preload_meta
                self.load();
                self.constructGroups();
            end
        end
        
        function load(self)
            % Some preprocessing
            self.validateStoreAttributes();
            self.group_lookup_table = self.buildLookupTable();
            
            % Build search set
            self.buildSearchSet();
        end
        
        function groups = getGroup(self, idx)
            arguments
                self
                idx=(1:self.num_groups)-1
            end
            % Create lazy loader if not present
            if isempty(self.groups)
                self.constructGroups();
            end
            idx = idx + 1;
            groups = self.groups(idx);
            to_load = self.unloaded_groups(idx);
            % Load any groups that need loading still.
            groups(to_load).load_all();
            self.unloaded_groups(idx) = false;
        end
        
        function search_set = getSearchSet(self)
            if isempty(self.search_set)
                self.buildSearchSet();
            end
            search_set = self.search_set;
        end
    end
    
    methods (Access=protected)
        function validateStoreAttributes(self)
            % Get the proposed attributes
            pa = cell2struct({self.info.Attributes.Value},{self.info.Attributes.Name},2);
            % Validate that this is a collection group
            % Build the default attributes
            da.search_set = [];
            da.num_prefix = 1;

            % Merge in
            fields = fieldnames(pa).';
            for fieldname = fields
                da.(fieldname{1}) = pa.(fieldname{1});
            end

            % Make sure they make sense
            prefixes_found = sum(matches(fields, "prefix" + digitsPattern),'all');
            if da.num_prefix ~= prefixes_found
                error(['Malformed ReachSet, num_prefixes != prefixes found for ', self.group_path])
            end

            % Store the attributes
            self.attributes = da;
        end
        
        function group_lookup_table = buildLookupTable(self)
            % Prepare the group id lookup table
            % Positive values -> Groups, negative values -> Dataset
            % + 1 if we have a search set string
            ssn = self.attributes.search_set;
            add_search = ~isempty(ssn) && (ischar(ssn) || isstring(ssn));
            group_lookup_table = zeros(self.num_groups + double(add_search), self.attributes.num_prefix);
            % Isolate out names and lookup id's
            % Little trick to ensure it remains cells
            group_names = {};
            if ~isempty(self.info.Groups)
                [~, group_names, ~] = fileparts({self.info.Groups.Name, ''});
                group_names = group_names(1:end-1);
            end
            group_name_ids = 1:length(group_names);
            dataset_names = {};
            if ~isempty(self.info.Datasets)
                dataset_names = {self.info.Datasets.Name};
            end
            dataset_name_ids = 1:length(dataset_names);
            % For each of the prefixes, proceed
            for i=1:self.attributes.num_prefix
                % Get what we are searching
                prefix = self.attributes.(['prefix', num2str(i-1)]);
                pattern = prefix + digitsPattern;
                id_start = length(prefix) + 1;
                % Process for Groups
                group_mask = matches(group_names, pattern);
                reverse_ids = group_name_ids(group_mask);
                set_ids = cellfun(@(name)str2double(name(id_start:end)), group_names(group_mask)) + 1;
                group_lookup_table(set_ids, i) = reverse_ids;
                % Process for Datasets
                dataset_mask = matches(dataset_names, pattern);
                reverse_ids = dataset_name_ids(dataset_mask);
                set_ids = cellfun(@(name)str2double(name(id_start:end)), dataset_names(dataset_mask)) + 1;
                group_lookup_table(set_ids, i) = -reverse_ids;
            end
            % If we have the search set string, index that as well
            if add_search
                set_id = self.num_groups + 1;
                % Process for Groups
                [~, name, ~] = fileparts(ssn);
                group_mask = matches(group_names, name);
                reverse_id = group_name_ids(group_mask);
                if ~isempty(reverse_id)
                    group_lookup_table(set_id, 1) = reverse_id;
                end
                % Process for Datasets
                dataset_mask = matches(dataset_names, name);
                reverse_id = dataset_name_ids(dataset_mask);
                if ~isempty(reverse_id)
                    group_lookup_table(set_id, 1) = -reverse_id;
                end
            end
        end
        
        function constructGroups(self)
            % Setup all the GroupLoaders
            all_groups = size(self.group_lookup_table, 1);
            groups_arr(all_groups) = loaders.RSLoader.GroupLoader;
            [groups_arr.loader] = deal(self.loader);
            [groups_arr.parent] = deal(self);
            for i=1:all_groups
                groups_arr(i).indices = self.group_lookup_table(i, :);
                groups_arr(i) = groups_arr(i).construct();
            end
            self.groups = groups_arr;
            self.unloaded_groups = true(1, all_groups);
            % But they're all loaded if we preload the metadata
            if self.loader.preload_meta
                self.unloaded_groups = ~self.unloaded_groups;
            end
        end
        
        function buildSearchSet(self)
            search_set_name = self.attributes.search_set;
            if isempty(search_set_name) || ischar(search_set_name) || isstring(search_set_name)
                all_groups = size(self.group_lookup_table, 1);
                self.search_set = self.getGroup(all_groups-1);
            else
                % Lazy iterate over all groups for this... slow for now
                % Current assumes all terminal groups. okay for now,
                % rework later
                self.search_set = loaders.RSLoader.StripeZonoGroup(self, search_set_name);
            end
        end
    end
    
    methods
        function group_path = get.group_path(self)
            group_path = self.info.Name;
        end
        function num_sets = get.num_groups(self)
            num_sets = self.attributes.num_groups;
        end
    end
end
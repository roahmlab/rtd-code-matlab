classdef CollectionGroup < handle
    properties (Access=private)
        parent_loader
        group_path
        do_caching = false
        
        prefix
        search_set
    end
    properties
        num_groups
        groups
    end
    methods
        function self = CollectionGroup(parent_loader, group_path, options)
            arguments
                parent_loader
                group_path
                options.do_caching = false
            end
            self.parent_loader = parent_loader;
            self.group_path = group_path;
            self.do_caching = options.do_caching;
            
            % Save the metadata for the current
            if self.get_attribute('num_prefix') ~= 1
                error('Unexpected Format!')
            end
            self.prefix = char(self.get_attribute('prefix0'));
            self.num_groups = self.get_attribute('num_groups');
            search_set_name = self.get_attribute('search_set');
            if isempty(search_set_name)
                self.search_set = [];
                return
            elseif ischar(search_set_name) || isstring(search_set_name)
                termgroup = temp.TerminalGroup(self.parent_loader, [self.group_path, '/', char(search_set_name)]);
                self.search_set = termgroup.getZonos();
            else
                % Lazy iterate over all groups for this... slow for now
                % Current assumes all terminal groups. okay for now,
                % rework later
                self.search_set = cell(1, self.num_groups);
                for i=1:self.num_groups
                    termgroup = temp.TerminalGroup(self.parent_loader, [self.group_path, '/', self.prefix, num2str(i-1)]);
                    self.search_set(i) = termgroup.getZonos(search_set_name);
                end
            end
        end
        
        function zonos = getSearchSet(self)
            zonos = self.search_set;
        end
        
        function groups = getGroup(self, idx)
            arguments
                self
                idx=(1:self.num_groups)-1
            end
            
            idx = idx(:);
            groups = cell(1, length(idx));
            basename = [self.group_path, '/', self.prefix];
            for i=1:length(groups)
                % get the right group
                propname = [basename, num2str(idx(i))];
                try
                    groups{i} = temp.TerminalGroup(self.parent_loader, propname);
                catch
                    groups{i} = temp.CollectionGroup(self.parent_loader, propname);
                end
            end
        end
        
        function attr = get_attribute(self, attr_name)
            attr = self.parent_loader.get_attribute(self.group_path, attr_name);
        end
        
        function rawdata = get_data(self, range)
            arguments
                self
                range=[indices(1) indices(end)]
            end
            rawdata = self.parent_loader.get_data(self.group_path, range);
        end
    end
end
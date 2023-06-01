classdef TerminalGroup < handle
    properties (Access=private)
        parent_loader
        group_path
        do_caching = false
        
        indices
        data
    end
    properties
        num_sets
        zonos
    end
    methods
        function self = TerminalGroup(parent_loader, group_path, options)
            arguments
                parent_loader
                group_path
                options.do_caching = false
            end
            self.parent_loader = parent_loader;
            self.group_path = group_path;
            self.do_caching = options.do_caching; %TODO
            
            % Save the num sets for the current
            self.num_sets = self.get_attribute('num_sets');
            self.indices = self.get_attribute('indices');
        end
        
        function zonos = getZonos(self, idx)
            arguments
                self
                idx=[]
            end
            % Get all zonos if empty
            if isempty(idx)
                rawdata = self.get_data();
                zonos = arrayfun( ...
                            @(i)zonotope(rawdata(:,self.indices(i)+1:self.indices(i+1))), ...
                            1:self.num_sets, ...
                            UniformOutput=false);
                return
            end
            
            % Otherwise expand the idx's so we can get start and ends
            starts = idx(:);
            ends = starts + 1;
            num = length(starts);
            
            starts = self.indices(starts + 1);
            ends = self.indices(ends + 1);
            [lower, upper] = bounds([starts; ends]);
            % Get the rawdata and create the zonotopes
            rawdata = self.get_data([lower upper]);
            zonos = arrayfun( ...
                        @(i)zonotope(rawdata(:,starts(i)-lower+1:ends(i)-lower)), ...
                        1:num, ...
                        UniformOutput=false);
        end
        
        function attr = get_attribute(self, attr_name)
            attr = self.parent_loader.get_attribute(self.group_path, attr_name);
        end
        
        function rawdata = get_data(self, range)
            arguments
                self
                range=[self.indices(1) self.indices(end)]
            end
            rawdata = self.parent_loader.get_data(self.group_path, range);
        end
    end
end
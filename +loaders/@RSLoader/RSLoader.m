classdef RSLoader < loaders.RSLoader.CollectionGroup & loaders.RSLoader.DataLoader & handle
    % First draft code
    properties
        do_caching = false
        preload_data = false
        preload_meta = false
    end
    properties (Dependent)
        filename
    end
    
    methods
        function self = RSLoader(filename, options)
            arguments
                filename {mustBeFile}
                options.do_caching (1,1) logical = false
                options.preload_data (1,1) logical = false
                options.preload_meta (1,1) logical = true
            end
            % This is a gross hack
            dummystruct.do_caching = false;
            dummystruct.preload_data = false;
            dummystruct.preload_meta = false;
            self = self@loaders.RSLoader.CollectionGroup(dummystruct, h5info(filename));
            self.loader = self;
            % This is a gross hack
            
            % Finish
            self.do_caching = options.do_caching; % TODO
            self.preload_data = options.preload_data; % TODO
            self.preload_meta = options.preload_meta; % TODO

            self.load();
            self.constructGroups();
        end

        function data = get_data(self, group_path, start_idx, end_idx)
            arguments
                self loaders.RSLoader
                group_path {mustBeTextScalar}
                start_idx (1,2) double
                end_idx (1,2) double
            end
            data = h5read(self.filename, group_path, start_idx, end_idx);
        end
    end
    
    methods
        function filename = get.filename(self)
            filename = self.info.Filename;
        end
    end
end
            
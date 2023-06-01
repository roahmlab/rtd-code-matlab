classdef RSLoader < handle
    % First draft code
    properties
        filename
        do_caching = false
    end
    
    methods
        function self = RSLoader(filename, options)
            arguments
                filename {mustBeTextScalar}
                options.do_caching logical = false
            end
            self.filename = filename;
            self.do_caching = options.do_caching; % TODO
        end
        
        function group = getRootGroup(self)
            group = temp.CollectionGroup(self, '/');
        end

        function attr = get_attribute(self, group_path, attr_name)
            attr = h5readatt(self.filename, group_path, attr_name);
        end

        function data = get_data(self, group_path, range)
            % Should rewrite everything with h5info's info!
            info = h5info(self.filename, group_path);
            rows = info.Dataspace.Size(1);
            range = double(range);
            data = h5read(self.filename, group_path, [1, range(1)+1], [rows, range(2)]);
        end
    end
end
            
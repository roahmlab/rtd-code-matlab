classdef DataLoader < handle
    properties (Abstract)
        filename
        do_caching
        preload_data
        preload_meta
    end
    methods (Abstract)
        get_data(self, identifier, start_idx, end_idx)
    end
end
classdef RangeScaler
    properties
        in_range(:,2) double
        out_range(:,2) double
    end
    properties (Dependent)
        num_parameters(1,1) double
    end

    methods
        function self = RangeScaler(in_range, out_range)
            arguments
                in_range(:,2) double
                out_range(:,2) double
            end
            self.in_range = in_range;
            self.out_range = out_range;
        end
        function scaled_out = scaleout(self, parameters)
            arguments
                self(1,1) rtd.util.RangeScaler
                parameters(:,1) double
            end
            scaled_out = rescale(parameters, ...
                self.out_range(:,1), ...
                self.out_range(:,2), ...
                'InputMin', self.in_range(:,1), ...
                'InputMax', self.in_range(:,2));
        end
        function scaled_in = scalein(self, parameters)
            arguments
                self(1,1) rtd.util.RangeScaler
                parameters(:,1) double
            end
            scaled_in = rescale(parameters, ...
                self.in_range(:,1), ...
                self.in_range(:,2), ...
                'InputMin', self.out_range(:,1), ...
                'InputMax', self.out_range(:,2));
        end
        function num_parameters = get.num_parameters(self)
            num_parameters = size(self.in_range);
            num_parameters = num_parameters(1);
        end
    end
end

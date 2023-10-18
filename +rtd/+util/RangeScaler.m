classdef RangeScaler
% Scales parameters from one range to another.
%
% This is a utility class for scaling parameters from one range to another.
% It stores the input and output ranges and provides methods for scaling
% parameters from the input to the output range (scaleout) and from the
% output to the input range (scalein).
%
% --- More Info ---
% Author: Adam Li (adamli@umich.edu)
% Written: 2023-09-07
%
% See also: rescale
%
% --- More Info ---
%

    properties
        % The input range of the parameters. The first column is the lower
        % bound and the second column is the upper bound.
        in_range(:,2) double

        % The output range of the parameters. The first column is the lower
        % bound and the second column is the upper bound.
        out_range(:,2) double
    end

    properties (Dependent)
        % The number of parameters to be scaled.
        num_parameters(1,1) double
    end

    methods
        function self = RangeScaler(in_range, out_range)
            % Constructor for the RangeScaler class.
            %
            % Arguments:
            %   in_range (double): The input range of the parameters. The
            %       first column is the lower bound and the second column
            %       is the upper bound.
            %   out_range (double): The output range of the parameters.
            %       The first column is the lower bound and the second
            %       column is the upper bound.
            %
            arguments
                in_range(:,2) double
                out_range(:,2) double
            end

            self.in_range = in_range;
            self.out_range = out_range;
        end

        function scaled_out = scaleout(self, parameters)
            % Scales parameters from the input range to the output range.
            %
            % Arguments:
            %   parameters (double): The parameters to be scaled. The
            %       length of the vector must match the number of parameters.
            %
            % Returns:
            %   scaled_out (double): The scaled parameters.
            %
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
            % Scales parameters from the output range to the input range.
            %
            % Arguments:
            %   parameters (double): The parameters to be scaled. The
            %       length of the vector must match the number of parameters.
            %
            % Returns:
            %   scaled_in (double): The scaled parameters.
            %
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
    end
    
    % methods for dependent properties
    methods
        function num_parameters = get.num_parameters(self)
            num_parameters = size(self.in_range);
            num_parameters = num_parameters(1);
        end
    end
end

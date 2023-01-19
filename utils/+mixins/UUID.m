classdef UUID < handle
% This mixin provides a unique identifier for every "hard instance."
%
% It generates a UUID to assist with references to this object. If a given
% object is also stored in a table or similar, this acts as a clearly
% unique identifier for lookups and vice versa.
%
% Warning:
%     If you generate an array of a class derived from this through last
%     object placement in MATLAB, all prior objects in that array will
%     share the same UUID due to how MATLAB initializes arrays of handle
%     objects.
%     https://www.mathworks.com/help/matlab/matlab_oop/initializing-arrays-of-handle-objects.html
%

    properties
        % A character-array string representing the UUID generated.
        uuid
    end
    methods
        function self = UUID()
            % Constructs the UUID mixin.
            %
            % This default constructor is run for any classes that inherit
            % from this mixin. It will generate and associate a UUID to
            % the `uuid` property of this class.
            %
            self.uuid = char(matlab.lang.internal.uuid());
        end
    end
end
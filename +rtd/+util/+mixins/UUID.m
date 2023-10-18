classdef UUID < handle
% This mixin provides a unique identifier for every "hard instance."
%
% It generates a UUID to assist with references to this object. If a given
% object is also stored in a table or similar, this acts as a clearly
% unique identifier for lookups and vice versa.
%
% --- More Info ---
% Author: Adam Li (adamli@umich.edu)
% Written: 2022-10-05
% Last Revised: 2023-09-12 (Adam Li)
%
% See also: rtd.functional.uuid
%
% --- Revision History ---
% 2023-09-12 (Adam Li): Created an internal noncopyable property to store the UUID to ensure each hard instance can generate a unique UUID.
%
% --- More Info ---
%

    properties (NonCopyable, Access=private)
        % A character-array string representing the UUID generated.
        rtd_internal_uuid(1,:) char
    end
    properties (Dependent)
        % A character-array string representing the UUID generated for this object.
        uuid(1,:) char
    end
    methods
        function self = UUID()
            % Constructs the UUID mixin.
            %
            % This default constructor is run for any classes that inherit
            % from this mixin. It will generate and associate a UUID to
            % the `uuid` property of this class.
            %
            self.rtd_internal_uuid = rtd.functional.uuid();
        end

        function uuid = get.uuid(self)
            % If the UUID has not been generated, generate it.
            if isempty(self.rtd_internal_uuid)
                self.rtd_internal_uuid = rtd.functional.uuid();
            end
            % return the UUID
            uuid = self.rtd_internal_uuid;
        end
    end
end
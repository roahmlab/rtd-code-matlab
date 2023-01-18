classdef UUIDbase < handle
    properties
        uuid
    end
    methods
        function self = UUIDbase()
            self.uuid = char(matlab.lang.internal.uuid());
        end
    end
end
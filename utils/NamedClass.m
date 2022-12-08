classdef NamedClass < handle
    properties
        classname char = ''
        verbose_level {mustBeScalarOrEmpty} = 0
    end
    methods
        function self = NamedClass()
            self.classname = metaclass(self).Name;
        end
        function vdisp(self, output, level)
            % Display a string s if the message's verbose level is greater
            % than or equal to the planner's verbose level.
            if nargin < 3
                level = 0 ;
            end
            if self.verbose_level >= level
                name = self.classname;
                if isprop(self, "name")
                    name = [char(self.name), '-', self.classname];
                end
                disp([name, ': ', char(output)])
            end
        end
    end
end
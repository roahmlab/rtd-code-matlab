classdef NamedClass < handle
    properties
        name char = ''
        classname char = ''
        verboseLevel(1,1) LogLevel = LogLevel.OFF
    end
    methods
        function self = NamedClass()
            self.classname = metaclass(self).Name;
        end
        function vdisp(self, output, level, wait)
            arguments
                self
                output {mustBeTextScalar}
                level(1,1) LogLevel = LogLevel.DEBUG
                wait(1,1) logical = false
            end
            % Display a string s if the message's verbose level is greater
            % than or equal to the planner's verbose level.
            % https://www.section.io/engineering-education/how-to-choose-levels-of-logging/
            if level >= self.verboseLevel
                name = self.classname;
                if ~isempty(self.name)
                    name = [char(self.name), '-', self.classname];
                end
                name = [name, '-', char(level)];
                disp([name, ': ', char(output)])
                % Pause if we want to wait after the message
                if wait
                    pause
                end
            end
        end
        function set_vdisplevel(self, level)
            arguments
                self
                level(1,1) LogLevel
            end
            % Change the verbose level
            self.verboseLevel = level;
        end
    end
end
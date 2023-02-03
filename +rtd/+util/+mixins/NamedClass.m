classdef NamedClass < handle
% This mixin automatically generates names for any class and adds verbose output
%
% It pulls the classname and packagename the class is under and saves them.
% With those saved names, it has a verbose logging capability which we can
% specify levels of verbosity for. It follows a subset of the log4j log
% levels defined in :mat:class:`+rtd.+util.+types.LogLevel`. The log levels can
% also be set by its name string.
%
% --- More Info ---
% Author: Adam Li (adamli@umich.edu)
% Written: 2022-10-05
% Last Revised: 2023-01-30 (Adam Li)
%
% See also rtd.util.types.LogLevel
%
% --- More Info ---
%

    properties
        % An optional name property which is appended to the log output
        name char = ''
        
        % The classname is generated from the metaclass of the class that uses this mixin
        classname char = ''
        
        % The packagename is generated from the metaclass of the class that uses this mixin
        packagename char = ''
        
        % Option to include the packagename with the logging output. If
        % true, the packagename is prepended to the classname before
        % output.
        show_packagename(1,1) logical = true
        
        % The tab level to use as the output. 0 indicates no tabs.
        tablevel(1,1) uint8 = 0
        
        % Log level of the class.
        verboseLevel(1,1) rtd.util.types.LogLevel = 'OFF'
    end
    
    methods
        function self = NamedClass()
            % Constructs the NamedClass mixin
            %
            % This default constructor is run for any classes that inherit
            % from this mixin. It extracts the current classname and
            % package name from the metaclass and saves it.
            %
            
            metaclass_data = metaclass(self);
            if ~isempty(metaclass_data.ContainingPackage)
                self.packagename = metaclass_data.ContainingPackage.Name;
                self.classname = extractAfter(metaclass_data.Name, length(self.packagename)+1);
            else
                self.classname = metaclass_data.Name;
            end
        end
        
        function vdisp(self, output, level, options)
            % Verbose display function
            %
            % Display a string s if the message's verbose level is greater
            % than or equal to the object's verbose level.
            % https://www.section.io/engineering-education/how-to-choose-levels-of-logging/
            %
            % Arguments:
            %     output (String): The string to display if the log level is verbose enough
            %     level (String, rtd.util.types.LogLevel): The level of the message
            %     options: Keyword arguments. See Below.
            % 
            % Keyword Arguments:
            %     wait (logical): Whether or not to pause execution after displaying the message
            %     show_packagename (logical): Overrides the show_packagename property of the class
            %     tablevel (uint8): Overrides the tablevel property of the class
            %
            arguments
                self rtd.util.mixins.NamedClass
                output {mustBeTextScalar}
                level(1,1) rtd.util.types.LogLevel = 'GENERAL'
                options.wait(1,1) logical = false
                options.show_packagename(1,1) logical = self.show_packagename
                options.tablevel(1,1) uint8 = self.tablevel
            end
            
            if level >= self.verboseLevel
                % Generate the prefix
                disp_name = self.classname;
                if options.show_packagename && ~isempty(self.packagename)
                    disp_name = [self.packagename, '.', disp_name];
                end
                if ~isempty(self.name)
                    disp_name = [self.name, '-', disp_name];
                end
                disp_name = [disp_name, '-', char(level)];
                % Add the tablevel
                disp_name = [repmat('  ', 1, options.tablevel), disp_name];
                % Add the message
                disp([disp_name, ': ', char(output)])
                % Pause if we want to wait after the message
                if options.wait
                    pause
                end
            end
        end
        
        function set_vdisplevel(self, level)
            % Set the verbosity of the current class's output
            %
            % Arguments:
            %     level (rtd.util.types.LogLevel): The level of the current class
            %
            arguments
                self rtd.util.mixins.NamedClass
                level(1,1) rtd.util.types.LogLevel
            end
            % Change the verbose level
            self.verboseLevel = level;
        end
        
        function vlevel = get_vdisplevel(self)
            % Get the verbosity of the current class's output
            %
            % Returns:
            %     rtd.util.types.LogLevel: The level of the current class
            %
            vlevel = self.verboseLevel;
        end
    end
end
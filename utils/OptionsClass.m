classdef OptionsClass < handle
    % OptionsClass
    % Mixin utility class that adds useful functionality for adding
    % many optional arguments to a class. It requires you to write a static
    % defaultoptions method.
    
    properties
        instanceOptions struct = struct()
    end
    
    methods(Abstract, Static)
        optionStruct = defaultoptions()
    end
    
    methods

        function self = OptionsClass()
            self.instanceOptions = self.defaultoptions();
        end
        
        function options = mergeoptions(self, newOptions)
            % mergeoptions - Merge in any number of option structs
            %
            % mergeoptions(OC, newOptions, ...) merges the new options into
            % any object inheriting from OptionsClass with an optional
            % return of a copy of the instance options struct.
            arguments
                self OptionsClass
            end
            arguments (Repeating)
                newOptions struct
            end
            
            for newOptionStruct = newOptions
                fields = fieldnames(newOptionStruct{1}).';
                for fieldname = fields
                    self.instanceOptions.(fieldname{1}) ...
                        = newOptionStruct{1}.(fieldname{1});
                end
            end
            if nargout
                options = self.instanceOptions;
            end
        end
        
        function options = getoptions(self)
            options = self.instanceOptions;
        end
    end
end
    
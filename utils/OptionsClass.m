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
                        = merge_or_replace(self.instanceOptions.(fieldname{1}), newOptionStruct{1}.(fieldname{1}));
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

function into = merge_or_replace(into, from)
    if isstruct(into) && isstruct(from)
        fields = fieldnames(from).';
        for fieldname = fields
            if isfield(into, fieldname{1})
                into.(fieldname{1}) = merge_or_replace(into.(fieldname{1}), from.(fieldname{1}));
            else
                into.(fieldname{1}) = from.(fieldname{1});
            end
        end
    else
        into = from;
    end
end

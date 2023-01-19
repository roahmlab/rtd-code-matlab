classdef Options < handle
% This mixin adds functionality for easy configuration of complex objects.
%
% The Options mixin introduces an `instanceOptions` property which holds
% a struct of all sorts of configuration options. It also includes utility
% code for the merging of these structs and is designed to be used with
% MATLAB arguments for Name-Value argument generation. This mixin requires 
% you to write a static `defaultoptions` method.
%
% Note:
%     This is best paired with a manual definition of the options in a
%     class's constructor as Name-Value arguments.
%     https://www.mathworks.com/help/matlab/matlab_prog/function-argument-validation-1.html#mw_1b62b6d6-a445-4c55-a9b9-9c70becfdbe6
%
    
    properties
        % A struct that holds the configuration options
        instanceOptions struct = struct
    end
    
    methods(Abstract, Static)
        % Abstract static method which needs to be implemented to provide
        % the default options for any given class. Should return a struct.
        optionStruct = defaultoptions()
    end
    
    methods

        function self = Options()
            % Constructs the Options mixin.
            %
            % This default constructor is run for any classes that inherit
            % from this mixin. It will call the required `defaultoptions()`
            % static function and create the initial `instanceOptions`.
            %
            self.instanceOptions = self.defaultoptions();
        end
        
        function options = mergeoptions(self, newOptions)
            % Merge in any number of option structs
            %
            % This method merges any number of new option structs into the
            % `instanceOptions` property with an optional return of a copy
            % of the `instanceOptions` struct.
            %
            % Arguments:
            %     newOptions (repeating): Struct with the configuration options to merge into the current `instanceOptions`.
            % 
            % Returns:
            %     struct: Optional copy of the merged `instanceOptions`.
            %
            
            arguments
                self mixins.Options
            end
            arguments (Repeating)
                newOptions struct
            end
            
            % go through each of the options and merge them, merging any
            % substructs of needed.
            for newOptionStruct = newOptions
                fields = fieldnames(newOptionStruct{1}).';
                for fieldname = fields
                    self.instanceOptions.(fieldname{1}) ...
                        = merge_or_replace(self.instanceOptions.(fieldname{1}), newOptionStruct{1}.(fieldname{1}));
                end
            end
            % Return an output if wanted.
            if nargout
                options = self.instanceOptions;
            end
        end
        
        function options = getoptions(self)
            % Returns the current `instanceOptions`
            %
            % This function can be overloaded to change how the
            % `instanceOptions` are returned if desired.
            %
            % Returns:
            %     struct: Copy of the current `instanceOptions`.
            options = self.instanceOptions;
        end
    end
end

% Utility function to merge substructs or replace the elements.
function into = merge_or_replace(into, from)
    % Merge if we're merging structs, otherwise just replace.
    if isstruct(into) && isstruct(from)
        fields = fieldnames(from).';
        for fieldname = fields
            % recursively call this function for each field that exists, or
            % just copy the data to fields that need to be created.
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

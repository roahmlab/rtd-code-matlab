classdef VarLogger < handle
% VarLogger is a modified Map which helps with logging step-wise data.
%
% It uses `containers.Map` as its underlying datatype, and can perform key
% validation and return flattened or stepwise data for analysis later or
% for summarizing.
%
% Warning:
%     The current documentation engine is unable to document the `get`
%     method of this class. Please check the source for how to use it!
%     https://github.com/sphinx-contrib/matlabdomain/issues/151
%
% --- More Info ---
% Author: Adam Li (adamli@umich.edu)
% Written: 2022-12-10
% Last Revised: 2023-01-18 (Adam Li)
%
% --- More Info ---
%

    properties
        % The Map which holds the data we have logged.
        log_entries containers.Map %
        
        % Whether or not to throw an error if a new key was used to add
        % more data.
        log_validateKeys logical %
    end
    
    methods
        function self = VarLogger(key_names, options)
            % Constructs the VarLogger container
            %
            % Keys can be added in the constructor, and an Name-Value
            % option to disable the validation of keys is present.
            %
            % Arguments:
            %     key_names (String - Repeating): Keys for values to log
            %     options: Keyword arguments. See Below.
            % 
            % Keyword Arguments:
            %     validate_keys (logical): Whether or not to throw an error when adding a new key to the log without explicitly using `addKey`.
            %
            arguments (Repeating)
                key_names {mustBeTextScalar}
            end
            arguments
                options.validate_keys(1,1) logical = true
            end
            
            % Say whether we should make sure the key exists first or just
            % add dynamically
            self.log_validateKeys = options.validate_keys;
            % Use a Map container so keys are hashed
            self.log_entries = containers.Map;
            
            % Add the keys
            self.addKeys(key_names{:});
        end
        
        function keys = keys(self)
            % Get the keys of the values that are logged
            %
            % Returns:
            %     cell(String): Cell array of strings for the keys.
            %
            keys = self.log_entries.keys;
        end
        
        function addKeys(self, key_names)
            % Adds keys to the current log.
            %
            % Arguments:
            %     key_names (String - Repeating): Keys for values to log
            %
            arguments
                self rtd.containers.VarLogger
            end
            arguments (Repeating)
                key_names {mustBeTextScalar}
            end
            
            % Determine the new keys, and add them
            keys_to_add = ~self.log_entries.isKey(key_names);
            for key=key_names(keys_to_add)
                self.log_entries(key{1}) = [];
            end
        end
        
        function add(self, key_name, value)
            % Adds entries to the log by repeating pairs of keys and values
            %
            % Arguments:
            %     key_name (String - Repeating): Key used to log the following value
            %     value (String - Repeating): Data to log
            %
            arguments
                self rtd.containers.VarLogger
            end
            arguments (Repeating)
                key_name {mustBeTextScalar}
                value
            end
            
            % validate entries
            keys_to_add = ~self.log_entries.isKey(key_name);
            if any(keys_to_add)
                temp = key_name(keys_to_add);
                % We want to know that we typo'd somewhere!
                if self.log_validateKeys
                    error('Log key %s does not exist!', temp{1})
                end
                % Otherwise, we'll just add new keys
                self.addKeys(temp{:});
            end
            
            % now append to the log for each key
            for kv_pair = [key_name; value]
                self.log_entries(kv_pair{1}) = [self.log_entries(kv_pair{1}), kv_pair(2)];
            end
        end

        function addStruct(self, struct_data)
            % Adds entries to the log based on a struct
            %
            % Arguments:
            %     struct_data (struct): Data to log as a struct
            %
            arguments
                self rtd.containers.VarLogger
                struct_data struct
            end
            args = namedargs2cell(struct_data);
            self.add(args{:});
        end
        
        function varargout = get(self, key_name, options)
            % Gets the log entries requested as either a struct or as multiple outputs
            %
            % Arguments:
            %     key_name (String - Repeating): Keys to retrieve log data for
            %     options: Keyword arguments. See Below.
            %
            % Keyword Arguments:
            %     flatten (logical): Whether or not the resulting output should be flattened or provided as logged cells
            %     as_struct (logical): Whether or not the resulting output should be in struct form
            %
            % Returns:
            %     struct or any: Struct holding the logged data or multiple output arguments with the requested log data.
            %
            arguments
                self rtd.containers.VarLogger
            end
            arguments (Repeating)
                key_name {mustBeTextScalar}
            end
            arguments
                options.flatten(1,1) logical = true
                options.as_struct(1,1) logical = true
            end
            
            % Validate the key names
            keys_to_add = ~self.log_entries.isKey(key_name);
            if any(keys_to_add)
                % We want to know that we typo'd somewhere!
                temp = key_name(keys_to_add);
                error('Log key %s does not exist!', temp{1})
            end
            
            % Get all keys requested
            if isempty(key_name)
                key_name = self.log_entries.keys;
            end
            % Number of keys requested
            num_res = length(key_name);
            
            % Make sure the output is the same size if there is more than
            % 1, or that it's as_struct is true for a single output
            if ~(nargout == num_res) && ~(nargout <= 1 && options.as_struct)
                error('Invalid log output option! Can only output as a single struct, or each individual log option requested!')
            end
            
            % Get the resulting values
            values = cell(1, num_res);
            for i=1:num_res
                entry = self.log_entries(key_name{i});
                if options.flatten && ~isempty(entry)
                    values{i} = [entry{:}];
                else
                    values{i} = {entry};
                end
            end
            
            % Output as requested (We did validation earlier)
            if nargout <= 1 && options.as_struct
                kv_pairs = [key_name; values];
                varargout{1} = struct(kv_pairs{:});
            else
                varargout = cellfun(@(x)x(:), values);
            end
        end
    end
end

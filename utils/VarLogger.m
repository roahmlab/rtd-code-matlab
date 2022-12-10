classdef VarLogger < handle
    % NOT A MIXIN
    
    properties
        log_entries containers.Map
        log_validateKeys logical
    end
    methods
        function self = VarLogger(validate_keys, key_names)
            arguments
                validate_keys logical = true
            end
            arguments (Repeating)
                key_names {mustBeTextScaler}
            end
            
            % Say whether we should make sure the key exists first or just
            % add dynamically
            self.log_validateKeys = validate_keys;
            % Use a Map container so keys are hashed
            self.log_entries = containers.Map;
            
            % Add the keys
            self.addKeys(key_names{:});
        end
        
        function keys = keys(self)
            keys = self.log_entries.key(key_names);
        end
        
        function addKeys(self, key_names)
            arguments
                self
            end
            arguments (Repeating)
                key_names {mustBeTextScaler}
            end
            
            % Determine the new keys, and add them
            keys_to_add = ~self.log_entries.isKey(key_names);
            for key=key_names(keys_to_add)
                self.log_entries(key{1}) = [];
            end
        end
        
        function add(self, key_name, value)
            arguments
                self
            end
            arguments (Repeating)
                key_name {mustBeTextScaler}
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
                self.log_entries(kv_pair{1}) = [self.log_entries(kv_pair{1}), kv_pair{2}];
            end
        end
        
        function varargout = get(self, flatten, as_struct, key_name)
            arguments
                self
                flatten logical = true
                as_struct logical = true
            end
            arguments (Repeating)
                key_name {mustBeTextScaler}
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
            if ~(nargout == num_res) && ~(nargout == 1 && as_struct)
                error('Invalid log output option! Can only output as a single struct, or each individual log option requested!')
            end
            
            % Get the resulting values
            values = cell(1, num_res);
            for i=1:num_res
                entry = self.log_entries(key_name{i});
                if flatten
                    values{i} = [entry{:}];
                else
                    values{i} = entry;
                end
            end
            
            % Output as requested (We did validation earlier)
            if as_struct
                kv_pairs = [key_name; values];
                varargout{1} = struct(kv_pairs{:});
            else
                [varargout{1:nargout}] = values{:};
            end
        end
    end
end

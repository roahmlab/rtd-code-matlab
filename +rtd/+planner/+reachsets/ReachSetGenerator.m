classdef ReachSetGenerator < rtd.util.mixins.NamedClass & handle
    % ReachableSets
    % This either encapsulates the reachable sets in memory, or enables the
    % online computation of reachable sets. It acts as a generator for a
    % single instance of ReachableSet
    properties (SetAccess=private)% Make private (Discuss)
        cache = {}
        cache_index = 0
        cache_size = 0
    end
    properties (Abstract)
        cache_max_size(1,1) double
    end
    methods (Abstract, Access=protected)
        % An example constructor, but can take anything needed
        %self = ReachableSets( ...
        %            robot ...
        %        )
        
        % Obtains the relevant reachable set for the robotstate provided
        % and outputs the singular instance of a reachable set.
        % Returns ReachbleSet
        reachableSet = generateReachableSet(self, robotState, varargin)
    end
    methods (Sealed)
        % This is the function that should be public for the class. It
        % handles how we actualy get the reachable set, with it calling
        % generateReachableSet if needed. This is useful if we need to use
        % one reachable set in another reachable set, and we don't want to
        % regenerate it online. It caches based on the uuid property, which
        % can be added to any object by adding UUIDbass as one of its base
        % classes.
        function reachableSet = getReachableSet(self, robotState, varargin)
            % if we don't want to use the cache or don't have a cache,
            % short circuit to generate.

            % Parse the inputs
            parser = inputParser;
            parser.KeepUnmatched = true;
            parser.PartialMatching = false;
            parser.StructExpand = false;
            parser.addParameter('ignore_cache', false);
            parser.parse(varargin{:})

            ignore_cache = parser.Results.ignore_cache;
            passthrough_args = namedargs2cell(parser.Unmatched);

            if ignore_cache || self.cache_max_size < 1
                reachableSet = generateReachableSet(self, robotState, passthrough_args{:});
                return
            end
            
            % Create hash of the input arguments
            hash = string(robotState.uuid);
            for hash_arg = 1:length(passthrough_args)
                try
                    hash = hash + strjoin(string(hash_arg));
                catch
                    % Ignore if not possible
                end
            end
            hash = char(hash);
            
            % search cache if it exists by generating reverse index and
            % iterating over the cache that exists.
            if self.cache_size > 0
                for idx=circshift((self.cache_size:-1:1), self.cache_index)
                    if strcmp(hash, self.cache{idx, 1})
                        reachableSet = self.cache{idx, 2};
                        return
                    end
                end
            end
            
            % if nothing else, create and add to cache
            % update indexing and cache size.
            self.cache_size = min(self.cache_size + 1, self.cache_max_size);
            self.cache_index = mod(self.cache_index, self.cache_max_size) + 1;
            
            % generate and store.
            reachableSet = generateReachableSet(self, robotState, passthrough_args{:});
            self.cache{self.cache_index, 1} = hash;
            self.cache{self.cache_index, 2} = reachableSet;
        end
    end
end
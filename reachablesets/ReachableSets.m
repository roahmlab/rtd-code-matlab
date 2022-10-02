classdef ReachableSets < handle
    % ReachableSets
    % This either encapsulates the reachable sets in memory, or enables the
    % online computation of reachable sets. It acts as a generator for a
    % single instance of ReachableSet
    properties % Make private
        cache = {}
        cache_index = 0
    end
    properties
        cache_size = 0
    end
    properties (Abstract)
        cache_max_size
    end
    methods (Abstract)
        % An example constructor, but can take anything needed
        %self = ReachableSets( ...
        %            robotInfo ...
        %        )
        
        % Obtains the relevant reachable set for the robotstate provided
        % and outputs the singular instance of a reachable set.
        % Returns ReachbleSet
        reachableSet = generateReachableSet(self, robotState, varargin)
    end
    methods
        function reachableSet = getReachableSet(self, robotState, ignore_cache, varargin)
            if ignore_cache || self.cache_size < 1
                reachableSet = generateReachableSet(self, robotState, varargin{:});
                return
            end
            
            % Create hash of the input arguments
            hash = robotState.uuid;
            for k = 1:nargin
                try
                    hash = hash + varargin{k}.uuid;
                catch
                    % Ignore if not a thing
                end
            end
            
            % search cache if it exists by generating reverse index and
            % iterating over the cache that exists.
            if self.cache_size > 0
                for idx=circshift((self.cache_size:-1:1), self.cache_index)
                    if hash == self.cache{idx, 1}
                        reachableSet = {idx, 2};
                        return
                    end
                end
            end
            
            % if nothing else, create and add to cache
            % update indexing and cache size.
            self.cache_size = min(self.cache_size + 1, self.cache_max_size);
            self.cache_index = mod(self.cache_index, self.cache_max_size) + 1;
            
            reachableSet = generateReachableSet(self, robotState, varargin{:});
            self.cache{self.cache_index, 1} = hash;
            self.cache{self.cache_index, 2} = reachableSet;
        end
    end
end
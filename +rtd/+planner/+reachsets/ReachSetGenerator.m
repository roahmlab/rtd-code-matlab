classdef ReachSetGenerator < rtd.util.mixins.NamedClass & handle
% Base class for the generation of reacheable sets for the planner
%
% The ReachSetGenerator class interfaces out the generation of reachable
% sets for the RTD planner. It contains a built-in cache which is enabled
% if `cache_max_size > 0`. Caching is based on the `uuid` property of the
% robot's state and any extra arguments passed through the
% `getReachableSet` method. This class can be used to encapsulate reachable
% sets generated offline, or the online computation of reachable sets. It
% acts as a generator for a single instance of ReachableSet
%
% Note:
%   Must have and define a `cache_max_size` property.
%   Must implement the `generateReachableSet(self, robotState, ...)`
%   method. See help for more information.
%
% --- More Info ---
% Author: Adam Li (adamli@umich.edu)
% Written: 2022-08-24
% Last Revised: 2023-02-01
%
% See also generateReachableSet, rtd.planner.reachsets.ReachSetInstance
%
% --- More Info ---
%

    % Interface components
    properties (Abstract)
        % The maximum size of the built-in cache. Set to 0 to disable.
        % Must be defined.
        cache_max_size(1,1) double
    end
    methods (Abstract, Access=protected)
        % Generate a new reachable set given a robot state and extra arguments if desired
        %
        % This method generates the relevant reachable set for the
        % `robotState` provided and outputs the singular instance of some
        % reachable set. It should always create a new instance of a
        % ReachableSetInstance.
        %
        % Arguments:
        %   robotState: Some entity state which the ReachableSetInstance is generated for.
        %   varargin: Additional optional arguments if desired. Will always be Name-Value arguments.
        % 
        % Returns:
        %   struct: A struct array containing the reachable set for the robot state
        %       provided, where reachableSet.rs is an instance of some derived version
        %       of `ReachSetInstance` and reachableSet.id is the id of the reachable set.
        %
        reachableSet = generateReachableSet(self, robotState, varargin)
    end

    % Caching utility. Allow visibility, but limit `set` ability
    properties (SetAccess=private)
        % The cache of the generator
        cache = {}

        % Current index of the cache
        cache_index = 0

        % Current size of the cache
        cache_size = 0
    end
    methods (Sealed)
        function reachableSet = getReachableSet(self, robotState, varargin)
            % Get a reachable set instance for the given robot state and passthrough arguments
            %
            % This function handles how to actually get the reachable set
            % with built in caching of already generated instances. It will
            % call `generateReachableSet` as needed. This is useful if we
            % need to use one reachable set in another reachable set and we
            % don't want to regenerate it online. It performs caching based
            % on the `uuid` property of the provided robotState and and the
            % string cast of any additional arguments. If we don't want to
            % use the cache on one call, setting the Name-Value argument
            % `ignore_cache` to true will bypass caching altogether.
            %
            % Arguments:
            %   robotState (rtd.entity.state.EntityState): Some state of used for generation / keying
            %   varargin: Keyword arguments (passed through). See below
            %
            % Keyword Arguments:
            %   ignore_cache (logical): If set true, the cache is completely ignored
            %   ... : Any remaining arugments to passthrough to `generateReachableSet`
            %
            % Returns:
            %   rtd.planner.reachsets.ReachSetInstance: The respective reachable set
            % 
            
            % Parse the inputs
            parser = inputParser;
            parser.KeepUnmatched = true;
            parser.PartialMatching = false;
            parser.StructExpand = false;
            parser.addParameter('ignore_cache', false);
            parser.parse(varargin{:})

            ignore_cache = parser.Results.ignore_cache;
            passthrough_args = namedargs2cell(parser.Unmatched);

            % if we don't want to use the cache or don't have a cache,
            % short circuit to generate.
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
            % verify the output
            names = fieldnames(reachableSet);
            if ~any([contains(names, 'id'), contains(names, 'rs')])
                error('Expected return with id and rs fields!')
            end
            self.cache{self.cache_index, 1} = hash;
            self.cache{self.cache_index, 2} = reachableSet;
        end
    end
end
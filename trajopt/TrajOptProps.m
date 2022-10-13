classdef TrajOptProps < handle
    % TrajOptProps
    % These are the consistent properties we use across RTD for trajectory
    % optimization, and they find themselves used all across the codebase.
    properties
        timeForCost
        planTime
        horizonTime
        doTimeout
        timeoutTime
        randomInit
    end
end
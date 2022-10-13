classdef RobotState < UUIDbase & handle
    % RobotState
    % The state of a generic robot at a point in time, which is saved.
    % Each hard instance of this (unique object, not seperate handles to
    % the same underlying object) will have a unique uuid.
    properties
        time
        % An relevant robot properties that evolve over time
    end
end
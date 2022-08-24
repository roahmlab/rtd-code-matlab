classdef WorldModel < handle
    %WORLDMODEL A model of the world used by RTD
    %   Similar to RobotModel, in that it contains a single initialization
    %   of WorldInfo, and then will create atomic instances of WorldState
    %   as needed.
    
    properties
        robots
    end
end


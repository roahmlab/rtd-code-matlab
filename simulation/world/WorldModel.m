classdef WorldModel < handle
    %WORLDMODEL A model of the world used by RTD
    %   Similar to RobotModel, in that it contains a single initialization
    %   of WorldInfo, and then will create atomic instances of WorldState
    %   as needed as well as evolve world dynamics/predictions.
    
    properties
        robots
    end
end


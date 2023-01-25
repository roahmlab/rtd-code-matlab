classdef SimulationState < uint8
% Enumeration type for identifying the current state of the simulation.
%
% --- More Info ---
% Author: Adam Li (adamli@umich.edu)
% Written: 2023-01-05
% Last Revised: 2023-01-20 (Adam Li)
%
% See also rtd.core.simulation.Simulation
%
% --- More Info ---
%
    enumeration
        INVALID         (0)
        CONSTRUCTED     (4)
        SETTING_UP      (8)
        SETUP_READY     (12)
        INITIALIZING    (16)
        READY           (20)
        PRE_STEP        (24)
        STEP            (28)
        POST_STEP       (32)
        COMPLETED       (36)
    end
    
    % Add validation for the comparison operators so we can compare to the
    % string equivalents
    methods
        function tf = lt(self, other)
            arguments
                self rtd.sim.types.SimulationState
                other rtd.sim.types.SimulationState
            end
            tf = lt@uint8(self, other);
        end
        function tf = gt(self, other)
            arguments
                self rtd.sim.types.SimulationState
                other rtd.sim.types.SimulationState
            end
            tf = gt@uint8(self, other);
        end
        function tf = le(self, other)
            arguments
                self rtd.sim.types.SimulationState
                other rtd.sim.types.SimulationState
            end
            tf = le@uint8(self, other);
        end
        function tf = ge(self, other)
            arguments
                self rtd.sim.types.SimulationState
                other rtd.sim.types.SimulationState
            end
            tf = ge@uint8(self, other);
        end
        function tf = ne(self, other)
            arguments
                self rtd.sim.types.SimulationState
                other rtd.sim.types.SimulationState
            end
            tf = ne@uint8(self, other);
        end
        function tf = eq(self, other)
            arguments
                self rtd.sim.types.SimulationState
                other rtd.sim.types.SimulationState
            end
            tf = eq@uint8(self, other);
        end
    end
end
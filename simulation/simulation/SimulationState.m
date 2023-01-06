classdef SimulationState < uint8
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
end
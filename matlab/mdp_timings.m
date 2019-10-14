
classdef mdp_timings
    properties
        DesiredDroneServerUpdateRate {mustBeNumeric}
        AchievedDroneServerUpdateRate {mustBeNumeric}
        MotionCaptureUpdateRate {mustBeNumeric}
        TimeToUpdateDrones {mustBeNumeric}
        WaitTimePerFrame {mustBeNumeric}
    end
end
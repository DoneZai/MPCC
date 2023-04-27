classdef Input < handle
    
    properties (Access = public)
        dThrottle
        dSteeringAngle
        dBrakes
        dVs
    end
    
    methods
        function obj = Input(dThrottle,dSteeringAngle,dBrakes,dVs)
            if nargin > 0
                obj.dThrottle = dThrottle;
                obj.dSteeringAngle = dSteeringAngle;
                obj.dBrakes = dBrakes;
                obj.dVs = dVs;
            end
        end
        
        function setZero(obj)
            obj.dThrottle = 0.0;
            obj.dSteeringAngle = 0.0;
            obj.dBrakes = 0.0;
            obj.dVs = 0.0;
        end
    end
end


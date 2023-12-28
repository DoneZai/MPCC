classdef Utilities
    
    properties (Access = private)
        log
        config
    end
    
    methods (Access = public)
        function obj = Utilities(config,log)
            obj.log = log;
            obj.config = config;
        end

        writeInputsToTxt(obj,filename) % write throttle, steeringAngle, brakes data to the .txt file

        writeInputsDerivativesToTxt(obj,filename) % write dThrottle, dSteeringAngle, dBrakes to the .txt file
    end
end


classdef Plot < handle

    properties (Access = private)
        config;
        parameters;
        track;
        log;
    end
    
    methods (Access = public)

        function obj = Plot(config,parameters,track,log)
            obj.config = config;
            obj.parameters = parameters;
            obj.track = track;
            obj.log = log;
        end
        
        carPositions(obj) % plot race track and car positions in X,Y coords on all horizons from log
       
        angles(obj) % plot frontSa, rearSa, sideSlipAngle and steeringAngle on all horizons from log
    
        boundedStates(obj) % plot state vector on all horizons from log
        
        boundedInputs(obj) % plot input vector on all horizons from log

        function stateVec = getStateVec(obj)
            stateVec = zeros(length(obj.log(1).x0),length(obj.log));
            for i = 1:length(obj.log)
                stateVec(:,i) = obj.log(i).x0;
            end
        end

    end

end


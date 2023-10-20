classdef TrackPoint
    properties
        xRef
        yRef
        dxRef
        dyRef
        thetaRef
        dthetaRef
    end
    
    methods
        function obj = TrackPoint(xRef,yRef,dxRef,dyRef,thetaRef,dthetaRef)
            obj.xRef = xRef;
            obj.yRef = yRef;
            obj.dxRef = dxRef;
            obj.dyRef = dyRef;
            obj.thetaRef = thetaRef;
            obj.dthetaRef = dthetaRef;
        end
    end
end


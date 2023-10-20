classdef Bounds
    %BOUNDS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        d_uBoundsX
        d_lBoundsX
        d_uBoundsU
        d_lBoundsU
        d_uBoundsS
        d_lBoundsS
    end
    
    methods (Access = public)
        function obj = Bounds(bounds)
            obj.d_uBoundsX = struct2array(bounds.upperStateBounds)';
            obj.d_lBoundsX = struct2array(bounds.lowerStateBounds)';
            obj.d_uBoundsU = struct2array(bounds.upperInputBounds)';
            obj.d_lBoundsU = struct2array(bounds.lowerInputBounds)';
            obj.d_uBoundsS = zeros(2,1);
            obj.d_lBoundsS = zeros(2,1);
        end

        function boundsLX = getBoundsLX(obj,x)
            boundsLX = obj.d_lBoundsX - stateToVector(x);
        end
        
        function boundsUX = getBoundsUX(obj,x) 
            boundsUX = obj.d_uBoundsX - stateToVector(x);
        end
        
        function boundsLU = getBoundsLU(obj,u)
            boundsLU = obj.d_lBoundsU - inputToVector(u);
        end
        
        function boundsUU = getBoundsUU(obj,u) 
            boundsUU = obj.d_uBoundsU - inputToVector(u);
        end
        
        function boundsLS = getBoundsLS(obj)
            boundsLS = obj.d_lBoundsS;
        end
        
        function boundsUS = getBoundsUS(obj)
            boundsUS = obj.d_uBoundsS;
        end
    end
end


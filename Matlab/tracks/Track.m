classdef Track  < handle  
    properties (Access = public)
        x
        y

        xInner
        yInner

        xOuter
        yOuter
    end
    
    methods (Access = public)
        function obj = Track(conesBlue,conesYellow,centerLine)
            if nargin > 2
                obj.xOuter = conesBlue(:,1);
                obj.yOuter = conesBlue(:,2);
                obj.xInner = conesYellow(:,1);
                obj.yInner = conesYellow(:,2);
                obj.x = centerLine(:,1);
                obj.y = centerLine(:,2);
            else
                obj.xOuter = conesBlue(:,1);
                obj.yOuter = conesBlue(:,2);
                obj.xInner = conesYellow(:,1);
                obj.yInner = conesYellow(:,2);
                obj.genCenterLine();
            end
        end
        
        function genCenterLine(obj)
            bSize = size(obj.xOuter,1);
            obj.x = zeros(bSize,1);
            obj.y = zeros(bSize,1);
            
            for i = 1:bSize
                coneYellowIndex = obj.getYellowConeIndex(i);
                obj.x(i) = (obj.xOuter(i) + obj.xInner(coneYellowIndex)) / 2.0;
                obj.y(i) = (obj.yOuter(i) + obj.yInner(coneYellowIndex)) / 2.0;
            end
        end

        function coneYellowIndex = getYellowConeIndex(obj,coneBlueIndex)
            ySize = size(obj.xInner,1);
            d = inf;
            for i = 1:ySize
                newD = hypot(obj.xOuter(coneBlueIndex) - obj.xInner(i),obj.yOuter(coneBlueIndex) - obj.yInner(i));
                if newD < d
                    d = newD;
                    coneYellowIndex = i;
                end
            end
        end
    end
end


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
        function obj = Track(conesBlue,conesYellow)
            obj.xOuter = conesBlue(:,1)';
            obj.yOuter = conesBlue(:,2)';
            obj.xInner = conesYellow(:,1)';
            obj.yInner = conesYellow(:,2)';
            obj.genCenterLine();
        end
        
        function genCenterLine(obj)
            bSize = size(obj.xOuter,2);
            obj.x = zeros(1,bSize);
            obj.y = zeros(1,bSize);
            
            for i = 1:bSize
                coneYellowIndex = obj.getYellowConeIndex(i);
                obj.x(i) = (obj.xOuter(i) + obj.xInner(coneYellowIndex)) / 2.0;
                obj.y(i) = (obj.yOuter(i) + obj.yInner(coneYellowIndex)) / 2.0;
            end
        end

        function coneYellowIndex = getYellowConeIndex(obj,coneBlueIndex)
            ySize = size(obj.xInner,2);
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


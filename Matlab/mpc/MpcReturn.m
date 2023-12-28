classdef MpcReturn
    properties (Access = public)
        x0
        u0
        mpcHorizon
        solverStatus
        cost
        circlesCenters
    end

    methods (Access = public)
        function obj = MpcReturn(x0,u0,mpcHorizon,solverStatus,cost,circlesCenters)
            if nargin > 0
                obj.x0 = x0;
                obj.u0 = u0;
                obj.mpcHorizon = mpcHorizon;
                obj.solverStatus = solverStatus;
                obj.cost = cost;
                obj.circlesCenters = circlesCenters;
            end
        end
    end

end


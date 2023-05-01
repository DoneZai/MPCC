classdef MPCReturn
    properties (Access = public)
        x0
        u0
        mpcHorizon
        stages
        solverStatus
    end

    methods (Access = public)
        function obj = MPCReturn(x0,u0,mpcHorizon,stages,solverStatus)
            if nargin > 0
                obj.x0 = x0;
                obj.u0 = u0;
                obj.mpcHorizon = mpcHorizon;
                obj.stages = stages;
                obj.solverStatus = solverStatus;
            end
        end
    end

end
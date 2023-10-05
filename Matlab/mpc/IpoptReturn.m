classdef IpoptReturn
    properties (Access = public)
        x0
        u0
        mpcHorizon
    end

    methods (Access = public)
        function obj = IpoptReturn(x0,u0,mpcHorizon)
            if nargin > 0
                obj.x0 = x0;
                obj.u0 = u0;
                obj.mpcHorizon = mpcHorizon;
            end
        end
    end

end


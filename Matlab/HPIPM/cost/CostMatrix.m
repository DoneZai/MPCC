classdef CostMatrix
    properties (Access = public)
        Q
        R
        S
        q
        r
        Z
        z
    end

    methods
        function obj = CostMatrix(Q,R,S,q,r,Z,z)
            if nargin > 0
                obj.Q = Q;
                obj.R = R;
                obj.S = S;
                obj.q = q;
                obj.r = r;
                obj.Z = Z;
                obj.z = z;
            end
        end
    end
end


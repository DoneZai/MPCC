classdef ConstraintsMatrix
    properties (Access = public)
        c
        d
        dl
        du
    end

    methods (Access = public)
        function obj = ConstraintsMatrix(c, d, dl, du)
            if nargin > 0
                obj.c = c;
                obj.d = d;
                obj.dl = dl;
                obj.du = du;
            end
        end
    end
end


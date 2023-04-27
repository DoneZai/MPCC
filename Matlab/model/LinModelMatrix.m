classdef LinModelMatrix
    properties
        a
        b
        g
    end
    
    methods
        function obj = LinModelMatrix(a,b,g)
            if nargin > 0
                obj.a = a;
                obj.b = b;
                obj.g = g;
            end
        end
    end
end


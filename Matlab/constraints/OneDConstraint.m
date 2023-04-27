classdef OneDConstraint
    
    properties (Access = public)
        cI
        dlI
        duI
    end
    
    methods (Access = public)
        function obj = OneDConstraint(cI,dlI,duI)
            obj.cI = cI;
            obj.dlI = dlI;
            obj.duI = duI;
        end
    end
end


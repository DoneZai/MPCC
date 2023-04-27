classdef OptVariables  
    properties (Access = public)
        xk
        uk
    end

    methods (Access = public)
        function obj = OptVariables()
            obj.xk = State();
            obj.uk = Input();
        end
    end
end
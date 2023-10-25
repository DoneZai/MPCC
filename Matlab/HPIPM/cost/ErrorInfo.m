classdef ErrorInfo
    properties
        error
        dError
    end
    
    methods
        function obj = ErrorInfo(error,dError)
            obj.error = error;
            obj.dError = dError;
        end
    end
end


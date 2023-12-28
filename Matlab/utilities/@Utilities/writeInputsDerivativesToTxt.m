function writeInputsDerivativesToTxt(obj,filename)
    inputsDerivatives = zeros(length(obj.log),obj.config.NU-1);

    for i = 1:length(obj.log)
        inputsDerivatives(i,:) = obj.log(i).mpcHorizon.inputs(1:3,1);
    end

    writematrix(inputsDerivatives,filename,'Delimiter',' ');
end


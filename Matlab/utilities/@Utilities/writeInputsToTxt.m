function writeInputsToTxt(obj,filename)
    inputs = zeros(length(obj.log),obj.config.NU-1);

    for i = 1:length(obj.log)
        inputs(i,:) = obj.log(i).mpcHorizon.states(8:10,1);
    end

    writematrix(inputs,filename,'Delimiter',' ');
end


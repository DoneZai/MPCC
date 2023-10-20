function stateVec = getStateVec(log)
    stateVec = zeros(length(log(1).x0),length(log));
        for i = 1:length(log)
            stateVec(:,i) = log(i).x0;
        end
end


function plotRace(log)
%%
    plotSize = size(log,2);

    stateVec = zeros(11,200);

    
    for i = 1:plotSize
        stateVec(:,i) = stateToVector(log(i).mpcHorizon(1).xk);
    end
    
    %figure(1);
    %plot(stateVec(1,:),stateVec(2,:),'g');

    frontSaVec = zeros(1,200);

    for i = 1:200
        frontSaVec(i) = atan2(stateVec(5,i) + parameters.car.lf * stateVec(6,i),stateVec(4,i)) - stateVec(9,i);
    end
    
    figure(2);
    hold on;
    plot(1:200, frontSaVec); 
    plot(1:200, stateVec(9,:));
    legend('frontSa','steeringAngle');
end
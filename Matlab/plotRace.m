function plotRace(log,track,trackCenter,parameters,config)
    plotSize = size(log,2);
    stateVec = zeros(config.NX,plotSize);
 
    for i = 1:plotSize
        stateVec(:,i) = stateToVector(log(i).mpcHorizon(1).xk);
    end
    
    %% race track and car state in X,Y coords
    figure(1);
    hold on;

    plot(track.xOuter,track.yOuter,'b');
    plot(track.xInner,track.yInner,'y');
    plot(trackCenter.x,trackCenter.y);
    plot(stateVec(1,:),stateVec(2,:),'g');
    legend('outer_border','inner_border','center_line','state');

    %% frontSa and steeringAngle
    figure(2);
    hold on;
    
    frontSaVec = zeros(1,plotSize);
    
    for i = 1:plotSize
        frontSaVec(i) = atan2(stateVec(5,i) + parameters.car.lf * stateVec(6,i),stateVec(4,i)) - stateVec(9,i);
    end
        
    plot(1:plotSize, frontSaVec); 
    plot(1:plotSize, stateVec(9,:));
    legend('frontSa','steeringAngle');
    
    %% throttle, steeringAngle and brakes
    figure(3);
    hold on;

    plot(1:plotSize, stateVec(8,:));
    plot(1:plotSize, stateVec(9,:));
    plot(1:plotSize, stateVec(10,:));
    legend('throttle','steeringAngle','brakes');

    %% s, vs
    figure(4);
    hold on;

    plot(1:plotSize, stateVec(7,:));
    plot(1:plotSize, stateVec(11,:));
    legend('s','vs');

    %% vs(s)
    figure(5)
    plot(stateVec(7,:),stateVec(11,:));
    legend('vs');

    %% horizon
    figure(6);
    hold on;
    
    plot(track.xOuter,track.yOuter,'b');
    plot(track.xInner,track.yInner,'y');
    plot(trackCenter.x,trackCenter.y);

    stateVec = zeros(config.NX,plotSize);
    
    for i = 1:plotSize
        stateVec(:,i) = stateToVector(log(i).mpcHorizon(1).xk);
    end

    plot(stateVec(1,:),stateVec(2,:),'g','LineWidth',1);

    for i = 1:plotSize
        horizonX = zeros(1,config.N);
        horizonY = zeros(1,config.N);
        for j = 1:config.N
            horizonX(j) = log(i).mpcHorizon(j).xk.x;
            horizonY(j) = log(i).mpcHorizon(j).xk.y;
        end
        plot(horizonX,horizonY);
    end

    %% steeringAngle
    figure(7);

    steeringAngle = zeros(1,plotSize);
    
    for i = 1:plotSize
        steeringAngle(1,i) = log(i).mpcHorizon(1).xk.steeringAngle;
    end

    plot(1:plotSize,steeringAngle);
    legend('steeringAngle');

    %% vx vy r throttle steeringAngle brakes
    figure(8)
    hold on;
 
    for i = 1:plotSize
        stateVec(:,i) = stateToVector(log(i).mpcHorizon(1).xk);
    end
        
    plot(1:plotSize,stateVec(4,:));
    plot(1:plotSize,stateVec(5,:));
    plot(1:plotSize,stateVec(6,:));
    plot(1:plotSize,stateVec(8,:));
    plot(1:plotSize,stateVec(9,:));
    plot(1:plotSize,stateVec(10,:));
    legend('vx','vy','r','throttle','steeringAngle','brakes');

    %% solver status
    figure(9);
    hold on;

    solverStatusVec = zeros(1,plotSize);

    for i = 1:plotSize
        solverStatusVec(i) = log(i).solverStatus;
    end
    plot(1:plotSize, solverStatusVec);
    
    %% Control inputs
    figure(10);
    hold on;

    inputVec = zeros(config.NU,plotSize);
    
    for i = 1:plotSize
        inputVec(:,i) = inputToVector(log(i).mpcHorizon(1).uk);
    end

    plot(1:plotSize,inputVec(1,:));
    plot(1:plotSize,inputVec(2,:));
    plot(1:plotSize,inputVec(3,:));
    plot(1:plotSize,inputVec(4,:));

    legend('dthrottle','dsteeringAngle','dbrakes','dvs');
end
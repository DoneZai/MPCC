function plotRace(log,track,trackCenter,parameters,config)
    
    %% race track and car state in X,Y coords
    figure(1);
    hold on;

    stateVec = zeros(length(log(1).x0),length(log));
    
    for i = 1:length(log)
        stateVec(:,i) = log(i).x0;
    end
   
    plot(track.xOuter,track.yOuter,'b');
    plot(track.xInner,track.yInner,'y');
    plot(trackCenter.x,trackCenter.y,'r');
    plot(stateVec(1,:),stateVec(2,:),'g');
    legend('outer_border','inner_border','center_line','state');

    %% frontSa, rearSa and steeringAngle
    figure(2);
    hold on;

    stateVec = log.mpcHorizon;
    plotSize = length(stateVec);
    
    frontSaVec = zeros(1,plotSize);
    rearSaVec = zeros(1,plotSize);
    
    for i = 1:plotSize
        frontSaVec(i) = atan2(stateVec(5,i) + parameters.car.lf * stateVec(6,i),stateVec(4,i)) - stateVec(9,i);
        rearSaVec(i) = atan2(stateVec(5,i) - parameters.car.lr * stateVec(6,i),stateVec(4,i));
    end
        
    plot(1:plotSize, frontSaVec);
    plot(1:plotSize, rearSaVec);
    plot(1:plotSize, stateVec(9,:));
    legend('frontSa','rearSa','steeringAngle');
    
    %% throttle, steeringAngle and brakes
    figure(3);
    hold on;

    stateVec = log.mpcHorizon;
    plotSize = length(stateVec);

    plot(1:plotSize, stateVec(8,:));
    plot(1:plotSize, stateVec(9,:));
    plot(1:plotSize, stateVec(10,:));
    legend('throttle','steeringAngle','brakes');

    %% s, vs
    figure(4);
    hold on;

    stateVec = log.mpcHorizon;
    plotSize = length(stateVec);

    plot(1:plotSize, stateVec(7,:));
    plot(1:plotSize, stateVec(11,:));
    legend('s','vs');

    %% vs(s)
    figure(5)

    stateVec = log.mpcHorizon;

    plot(stateVec(7,:),stateVec(11,:));
    legend('vs');

    %% horizon
    figure(6);
    hold on;

    stateVec = log.mpcHorizon;
    plotSize = length(stateVec);
    
    plot(track.xOuter,track.yOuter,'b');
    plot(track.xInner,track.yInner,'y');
    plot(trackCenter.x,trackCenter.y);

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

    steeringAngle = log.mpcHorizon(9);
    plotSize = length(stateVec);

    plot(1:plotSize,steeringAngle);
    legend('steeringAngle');

    %% vx vy r throttle steeringAngle brakes
    figure(8)
    hold on;

    stateVec = log.mpcHorizon;
    plotSize = length(stateVec);
  
    plot(1:plotSize,stateVec(4,:));
    plot(1:plotSize,stateVec(5,:));
    plot(1:plotSize,stateVec(6,:));
    plot(1:plotSize,stateVec(8,:));
    plot(1:plotSize,stateVec(9,:));
    plot(1:plotSize,stateVec(10,:));
    legend('vx','vy','r','throttle','steeringAngle','brakes');

    %% solver status
    %figure(9);
    %hold on;
%
    %plotSize = size(log,2);
    %solverStatusVec = zeros(1,plotSize);
%
    %for i = 1:plotSize
    %    solverStatusVec(i) = log(i).solverStatus;
    %end
    %plot(1:plotSize, solverStatusVec);
%
    %legend('solver status');
    
    %% Control inputs
    %figure(10);
    %hold on;
    %
    %plotSize = size(log,2);
    %inputVec = zeros(config.NU,plotSize);
    %
    %for i = 1:plotSize
    %    inputVec(:,i) = inputToVector(log(i).mpcHorizon(1).uk);
    %end
%
    %plot(1:plotSize,inputVec(1,:));
    %plot(1:plotSize,inputVec(2,:));
    %plot(1:plotSize,inputVec(3,:));
    %plot(1:plotSize,inputVec(4,:));
%
    %legend('dthrottle','dsteeringAngle','dbrakes','dvs');
end
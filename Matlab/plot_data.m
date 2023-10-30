    %% race track and car state in X,Y coords
    figure(1);
    hold on;

    stateVec = getStateVec(log);
   
    plot(track.xOuter,track.yOuter,'b');
    plot(track.xInner,track.yInner,'y');
    plot(trackCenter.x,trackCenter.y,'r');
    plot(stateVec(1,:),stateVec(2,:),'g');
    legend('outer_border','inner_border','center_line','state');

    %% frontSa, rearSa and steeringAngle
    figure(2);
    hold on;
    
    frontSaVec = zeros(1,length(log));
    rearSaVec = zeros(1,length(log));
    
    for i = 1:length(log)
        frontSaVec(i) = atan2(stateVec(5,i) + parameters.car.lf * stateVec(6,i),stateVec(4,i)) - stateVec(9,i);
        rearSaVec(i) = atan2(stateVec(5,i) - parameters.car.lr * stateVec(6,i),stateVec(4,i));
    end
        
    plot(1:length(log), frontSaVec);
    plot(1:length(log), rearSaVec);
    plot(1:length(log), stateVec(9,:));
    legend('frontSa','rearSa','steeringAngle');
    
    %% throttle, steeringAngle and brakes
    figure(3);
    hold on;

    plot(1:length(log), stateVec(8,:));
    plot(1:length(log), stateVec(9,:));
    plot(1:length(log), stateVec(10,:));
    legend('throttle','steeringAngle','brakes');

    %% s, vs
    figure(4);
    hold on;

    plot(1:length(log), stateVec(7,:));
    plot(1:length(log), stateVec(11,:));
    legend('s','vs');

    %% vs(s)
    figure(5)

    plot(stateVec(7,:),stateVec(11,:));
    legend('vs');

    %% horizons
    figure(6);
    hold on;

    plot(track.xOuter,track.yOuter,'b');
    plot(track.xInner,track.yInner,'y');
    plot(trackCenter.x,trackCenter.y, 'r');

    for i = 1:length(log)
        horizonX = log(i).mpcHorizon(1,:);
        horizonY = log(i).mpcHorizon(2,:);
        plot(horizonX,horizonY);
    end

    %% steeringAngle
    figure(7);

    plot(1:length(log),stateVec(9,:));
    legend('steeringAngle');

    %% vx vy r throttle steeringAngle brakes
    figure(8)
    hold on;
  
    plot(1:length(log),stateVec(4,:));
    plot(1:length(log),stateVec(5,:));
    plot(1:length(log),stateVec(6,:));
    plot(1:length(log),stateVec(8,:));
    plot(1:length(log),stateVec(9,:));
    plot(1:length(log),stateVec(10,:));
    legend('vx','vy','r','throttle','steeringAngle','brakes');

    %% solver status
    %figure(9);
    %hold on;
%
    %length(log) = size(log,2);
    %solverStatusVec = zeros(1,length(log));
%
    %for i = 1:length(log)
    %    solverStatusVec(i) = log(i).solverStatus;
    %end
    %plot(1:length(log), solverStatusVec);
%
    %legend('solver status');
    
    %% Control inputs
    %figure(10);
    %hold on;
    %
    %length(log) = size(log,2);
    %inputVec = zeros(config.NU,length(log));
    %
    %for i = 1:length(log)
    %    inputVec(:,i) = inputToVector(log(i).mpcHorizon(1).uk);
    %end
%
    %plot(1:length(log),inputVec(1,:));
    %plot(1:length(log),inputVec(2,:));
    %plot(1:length(log),inputVec(3,:));
    %plot(1:length(log),inputVec(4,:));
%
    %legend('dthrottle','dsteeringAngle','dbrakes','dvs');
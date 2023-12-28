function raceAngles(obj)
    f = figure(8);
    f.Name = 'raceAngles';
    f.NumberTitle = 'off';

    states = zeros(obj.config.NX,length(obj.log));

    for i = 1:length(obj.log)
        states(:,i) = obj.log(i).mpcHorizon.states(:,1);
    end

    tiledlayout(5,1); % steeringAngle, sideSlipAngle, kinematicSideSlipAngle, frontSlipAngle, rearSlipAngle

    % steeringAngle coordinate

    steeringAngle = nexttile;
    
    plot(steeringAngle,1:length(obj.log),states(9,:));

    ylim padded;
    yline(steeringAngle,obj.parameters.bounds.lowerStateBounds.steeringAngleL,'--red','steeringAngleL'); % lower bound
    yline(steeringAngle,obj.parameters.bounds.upperStateBounds.steeringAngleU,'--red','steeringAngleU'); % upper bound
    
    title(steeringAngle,'steeringAngle');
    ylabel(steeringAngle,'steeringAngle');

    % sideSlipAngle

    sideSlipAngle = nexttile;

    sideSlipAngles = zeros(length(obj.log),1);
    
    for i = 1:length(obj.log)
        vx = states(4,i);
        vy = states(5,i);
        sideSlipAngles(i) = atan2(vy,vx);
    end

    plot(sideSlipAngle,1:length(obj.log),sideSlipAngles);

    ylim padded;
    
    title(sideSlipAngle,'sideSlipAngle');
    ylabel(sideSlipAngle,'sideSlipAngle');

    % kinematicSideSlipAngle

    kinematicSideSlipAngle = nexttile;

    kinematicSideSlipAngles = zeros(length(obj.log),1);
    
    for i = 1:length(obj.log)
            vx = states(4,i);
            l = obj.parameters.car.lf+obj.parameters.car.lr;
            steeringAngle = states(9,i);
            kinematicSideSlipAngles(i) = vx*tan(steeringAngle)/l;
    end

    plot(kinematicSideSlipAngle,1:length(obj.log),kinematicSideSlipAngles);

    ylim padded;
    
    title(kinematicSideSlipAngle,'kinematicSideSlipAngle');
    ylabel(kinematicSideSlipAngle,'kinematicSideSlipAngle');

    % frontSlipAngle

    frontSlipAngle = nexttile;

    frontSlipAngles = zeros(length(obj.log),1);
    
    for i = 1:length(obj.log)
            vx = states(4,i);
            vy = states(5,i);
            r = states(6,i);
            steeringAngle = states(9,i);
            lf = obj.parameters.car.lf;
            frontSlipAngles(i) = atan2(vy+r*lf,vx)-steeringAngle;
    end

    plot(frontSlipAngle,1:length(obj.log),frontSlipAngles);

    ylim padded;
    yline(frontSlipAngle,-obj.parameters.mpcModel.maxAlpha,'--red','minAlpha'); % lower bound
    yline(frontSlipAngle,obj.parameters.mpcModel.maxAlpha,'--red','maxAlpha'); % upper bound
    
    title(frontSlipAngle,'frontSlipAngle');
    ylabel(frontSlipAngle,'frontSlipAngle');

    % rearSlipAngle

    rearSlipAngle = nexttile;

    rearSlipAngles = zeros(length(obj.log),1);
    
    for i = 1:length(obj.log)
            vx = states(4,i);
            vy = states(5,i);
            r = states(6,i);
            lr = obj.parameters.car.lr;
            rearSlipAngles(i) = atan2(vy-r*lr,vx);
    end

    plot(rearSlipAngle,1:length(obj.log),rearSlipAngles);

    ylim padded;
    yline(rearSlipAngle,-obj.parameters.mpcModel.maxAlpha,'--red','minAlpha'); % lower bound
    yline(rearSlipAngle,obj.parameters.mpcModel.maxAlpha,'--red','maxAlpha'); % upper bound
    
    title(rearSlipAngle,'rearSlipAngle');
    ylabel(rearSlipAngle,'rearSlipAngle');



end


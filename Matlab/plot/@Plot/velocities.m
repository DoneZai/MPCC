function velocities(obj)
    f = figure(9);
    f.Name = 'velocities';
    f.NumberTitle = 'off';

    states = zeros(obj.config.NX,length(obj.log));

    for i = 1:length(obj.log)
        states(:,i) = obj.log(i).mpcHorizon.states(:,1);
    end

    tiledlayout(3,1); % vx, vy, r

    % steeringAngle coordinate

    vx = nexttile;
    
    plot(vx,1:length(obj.log),states(4,:));

    ylim padded;
    yline(vx,obj.parameters.bounds.lowerStateBounds.vxL,'--red','vxMin'); % lower bound
    yline(vx,obj.parameters.bounds.upperStateBounds.vxU,'--red','vxMax'); % upper bound
    
    title(vx,'vx');
    ylabel(vx,'vx');

    % vy

    vy = nexttile;
    
    plot(vy,1:length(obj.log),states(5,:));

    ylim padded;
    yline(vy,obj.parameters.bounds.lowerStateBounds.vyL,'--red','vyMin'); % lower bound
    yline(vy,obj.parameters.bounds.upperStateBounds.vyU,'--red','vyMax'); % upper bound
    
    title(vy,'vy');
    ylabel(vy,'vy');

    % r

    r = nexttile;
    
    plot(r,1:length(obj.log),states(6,:));

    ylim padded;
    yline(r,obj.parameters.bounds.lowerStateBounds.rL,'--red','rMin'); % lower bound
    yline(r,obj.parameters.bounds.upperStateBounds.rU,'--red','rMax'); % upper bound
    
    title(r,'r');
    ylabel(r,'r');
end


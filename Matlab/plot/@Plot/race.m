function race(obj)
    f = figure(5);
    f.Name = 'race';
    f.NumberTitle = 'off';

    hold on;

    plot(obj.track.xOuter,obj.track.yOuter,'black');
    plot(obj.track.xInner,obj.track.yInner,'black');
    
    states = zeros(obj.config.NX,length(obj.log));

    for i = 1:length(obj.log)
        states(:,i) = obj.log(i).mpcHorizon.states(:,1);
    end

    horizonsPositions = zeros(2,obj.config.N+1,length(obj.log));

    for i = 1:length(obj.log)
        horizonsPositions(:,:,i) = obj.log(i).mpcHorizon.states(1:2,:);
    end

    plot(states(1,:),states(2,:),"green");

    for i = 1:length(obj.log)
        carBox = plotCarBox(states(:,i),1.0,2.0);
        horizonPositions = plotHorizonPositions(horizonsPositions(:,:,i));
        pause(0.05)
        %exportgraphics(gca,"parabola.gif","Append",true);
        delete(carBox);
        delete(horizonPositions);
    end
end


function carBox = plotCarBox(x0,w,l)
        car1 = x0(1:2) + [cos(x0(3))*l;sin(x0(3))*l] + [sin(x0(3))*w;-cos(x0(3))*w];
        car2 = x0(1:2) + [cos(x0(3))*l;sin(x0(3))*l] - [sin(x0(3))*w;-cos(x0(3))*w];
        car3 = x0(1:2) - [cos(x0(3))*l;sin(x0(3))*l] + [sin(x0(3))*w;-cos(x0(3))*w];
        car4 = x0(1:2) - [cos(x0(3))*l;sin(x0(3))*l] - [sin(x0(3))*w;-cos(x0(3))*w];

        carBox = plot([car1(1),car2(1),car4(1),car3(1),car1(1)],[car1(2),car2(2),car4(2),car3(2),car1(2)],'blue','LineWidth',0.5);
end

function horizonPositions = plotHorizonPositions(horizonsPositions)
    horizonPositions = plot(horizonsPositions(1,:),horizonsPositions(2,:),'red');
end


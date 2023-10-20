function testModel(mpc)
    u0 = zeros(4,1); % initial control input
    x0 = zeros(11,1); % initial state

    x0(4,1) = 20;
    u0(1,1) = 0;
    u0(2,1) = 0.01;
    u0(3,1) = 0.0;

    iterations = 800; % number of iterations to test
    
    X = zeros(length(x0),iterations);
    X(:,1) = x0;

    for i = 1:iterations-1
        X(:,i+1) = mpc.ode4(X(:,i),u0).full();
    end

    plotResult(X);
end

function plotResult(X)
    iterations = length(X(4,:));

    figure(1);
    hold on;
    plot(X(1,:),X(2,:),'g');
    legend('race_line');

    figure(2);
    hold on;
    plot(1:iterations,X(4,:));
    %plot(1:iterations,X(5,:));
    %plot(1:iterations,X(6,:));
    legend('vx','vy','r');
end


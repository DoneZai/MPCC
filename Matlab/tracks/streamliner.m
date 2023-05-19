%% script to streamline cone positions of the left and right bounds.
trackNameFile = 'mess.mat'; %input file path
load(trackNameFile);


% blue_cones streamlining
initialSize = size(cones_blue,1);
new_cones_blue = zeros(initialSize,2);
element = cones_blue(1,:);
new_cones_blue(1,:) = element;
cones_blue(1,:) = [];

nextElementIndex = 1;
for j = 2:initialSize-1
    minDist = inf;
    for i = 1:size(cones_blue,1)
        dist = sqrt((cones_blue(i,1) - element(1,1))^2 + (cones_blue(i,2) - element(1,2))^2);
        if minDist > dist
            minDist = dist;
            nextElementIndex = i;
        end
    end
    element = cones_blue(nextElementIndex,:);
    new_cones_blue(j,:) = element;
    cones_blue(nextElementIndex,:) = [];
end
new_cones_blue(end,:) = cones_blue(1,:);
cones_blue = new_cones_blue;

figure(1)
hold on;
plot(cones_blue(:,1), cones_blue(:,2),'b');

% yellow_cones streamlining
initialSize = size(cones_yellow,1);
new_cones_yellow = zeros(initialSize,2);
element = cones_yellow(1,:);
new_cones_yellow(1,:) = element;
cones_yellow(1,:) = [];

nextElementIndex = 1;
for j = 2:initialSize-1
    minDist = inf;
    for i = 1:size(cones_yellow,1)
        dist = sqrt((cones_yellow(i,1) - element(1,1))^2 + (cones_yellow(i,2) - element(1,2))^2);
        if minDist > dist
            minDist = dist;
            nextElementIndex = i;
        end
    end
    element = cones_yellow(nextElementIndex,:);
    new_cones_yellow(j,:) = element;
    cones_yellow(nextElementIndex,:) = [];
end
new_cones_yellow(end,:) = cones_yellow(1,:);
cones_yellow = new_cones_yellow;

plot(cones_yellow(:,1), cones_yellow(:,2),'y');

save tracks/new_mess.mat cones_blue cones_yellow % output file path


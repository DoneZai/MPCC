%% script to reverse cones positions of the right and left bounds.
oldTrackNameFile = 'mess.mat'; %track name
load(oldTrackNameFile);

cones_blue = flip(cones_blue);
cones_yellow = flip(cones_yellow);

save mess.mat cones_blue cones_yellow;
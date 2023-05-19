%% script to swap colcors of the track bounds.
trackNameFile = 'mess.mat'; %track name
load(trackNameFile);

old_blue_cones = cones_blue;
cones_blue = cones_yellow;
cones_yellow = old_blue_cones;

save mess.mat cones_blue cones_yellow;
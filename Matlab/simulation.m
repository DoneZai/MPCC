% Copyright (C) 2018, ETH Zurich, D-ITET, Kenneth Kuchera, Alexander Liniger
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
% 
%     http://www.apache.org/licenses/LICENSE-2.0
% 
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.
%% MPCC Simulation Script
clear
close all
clc

%% add spline library
addpath('constraints');
addpath('cost');
addpath('model');
addpath('mpc');
addpath('parameters');
addpath('simulator');
addpath('spline');
addpath('tracks');
addpath('types');

config = config();

% uncomment 33,34,36,37,39,40,42 to use FSG track

%trackNameFile = 'FSG.mat'; %track name
%load(trackNameFile);

%cones_blue(:,1) = cones_blue(:,1) + 30;
%cones_blue(:,2) = cones_blue(:,2) + 80;

%cones_yellow(:,1) = cones_yellow(:,1) + 30;
%cones_yellow(:,2) = cones_yellow(:,2) + 80;

%track = Track(cones_blue, cones_yellow);

%uncomment 46,47 to use MPCC track for the fullscale

[cones_blue,cones_yellow,center_line] = generateTestTrack();
track = Track(cones_blue,cones_yellow,center_line);

parameters = Parameters(config);

simulator = Simulator(parameters.car,parameters.tire);

mpc = Mpcc(config,parameters);
mpc.setTrack(track.x,track.y);
trackCenter = mpc.getTrack().getPath();
trackLength = mpc.getTrack().getLength();

phi0 = atan2(track.y(2) - track.y(1),track.x(2) - track.x(1));
x0 = State(track.x(1),track.y(1),phi0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
log(parameters.config.nSim) = MPCReturn();
for i = 1:parameters.config.nSim
    mpcSol = mpc.runMPC(copy(x0));
    x0 = simulator.simTimeStep(x0,mpcSol.u0,parameters.config.ts);
    x0.unwrap(trackLength);
    log(i) = mpcSol;
end

plotRace(log,track,trackCenter,parameters,config);

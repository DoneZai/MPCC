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
addpath('interface');
addpath('model');
addpath('mpc');
addpath('parameters');
addpath('simulator');
addpath('spline');
addpath('tracks');
addpath('types');

config = config();

trackNameFile = 'FSG.mat'; %track name
load(trackNameFile);

track = Track(cones_blue, cones_yellow);

parameters = Parameters(config);

% simulator
simulator = Simulator(parameters.car,parameters.tire);
%

mpc = Mpcc(config,parameters);
%profile on;
mpc.setTrack(track.x,track.y);
%profile viewer;
trackCenter = mpc.getTrack().getPath();

figure(1);
plot(track.xOuter,track.yOuter,'b');
hold on
plot(track.xInner,track.yInner,'y');
plot(track.x,track.y,'r')
plot(trackCenter.x,trackCenter.y);

phi0 = atan2(track.y(2) - track.y(1),track.x(2) - track.x(1));
%x0 = State(track.x(1),track.y(1),phi0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
x0 = State(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
log(200) = MPCReturn();
for i = 1:200
    mpcSol = mpc.runMPC(x0);
    x0 = simulator.simTimeStep(x0,mpcSol.u0,parameters.config.ts);
    log(i) = mpcSol;
end

plotRace(log);

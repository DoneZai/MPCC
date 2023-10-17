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

%addpath('/opt/casadi/');
addpath('/opt/casadi/')

import casadi.*;

config = config();

% uncomment 33,34,36,37,39,40,42 to use FSG track

trackNameFile = 'FSG.mat'; %track name
load(trackNameFile);

track = Track(cones_blue, cones_yellow);

%uncomment 46,47 to use MPCC track for the fullscale

%[cones_blue,cones_yellow,center_line] = generateTestTrack();
%track = Track(cones_blue,cones_yellow,center_line);

%uncomment 51,52,53,55,56,58,59,61,62,64 to use presplined FSG track from
%nirajbasnet

%trackOuterTable = readtable('track_outer.csv');
%trackInnerTable = readtable('track_inner.csv');
%trackCenterTable = readtable('track_center.csv');

%trackOuter(:,1) = trackOuterTable.Var1;
%trackOuter(:,2) = trackOuterTable.Var2;

%trackInner(:,1) = trackInnerTable.Var1;
%trackInner(:,2) = trackInnerTable.Var2;

%trackCenter(:,1) = trackCenterTable.Var1;
%trackCenter(:,2) = trackCenterTable.Var2;

%track = Track(trackOuter,trackInner,trackCenter);

parameters = Parameters(config);

simulator = DynamicSimulator(parameters.car,parameters.tire);
%simulator = KinematicSimulator(parameters.car,parameters.tire);

%mpc = Mpcc(config,parameters);
mpc = IpoptCasadi(config,parameters);
mpc.setTrack(track);
mpc.initMPC();
trackCenter = mpc.getTrack().getPath();

%figure(1);
%hold on;
%plot(track.xOuter,track.yOuter,'b');
%plot(track.xInner,track.yInner,'y');
%plot(trackCenter.x,trackCenter.y);
%legend('outer_border','inner_border','center_line');

trackPath = mpc.getTrack().getPath();
trackLength = mpc.getTrack().getLength();

phi0 = atan2(trackPath.y(2) - trackPath.y(1),trackPath.x(2) - trackPath.x(1));
x0 = [trackPath.x(1);trackPath.y(1);phi0;5;0;0;0;0;0;0;0];

%testModel(mpc);
%log(parameters.config.nSim) = MPCReturn();
%for i = 1:parameters.config.nSim
for i = 1:2
    mpcSol = mpc.runMPC(x0);
    x0 = simulator.simTimeStep(x0,mpcSol.u0,parameters.config.ts);
    log(i) = mpcSol;
end

plotRace(log,track,trackCenter,parameters,config);

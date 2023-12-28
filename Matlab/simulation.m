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

%% add subdirectories
addpath('model');
addpath('mpc');
addpath('parameters');
addpath('simulator');
addpath('spline');
addpath('tracks');
addpath('types');

%addpath('/opt/casadi/')
%% add subdirectories for the chosen solver

config = config();
parameters = Parameters(config);

if strcmp(config.solver,'ipopt')
    addpath('ipopt');
    mpc = Ipopt(config,parameters);
elseif strcmp(config.solver,'acados')
    addpath('acados/');
    mpc = Acados(config,parameters);
else
    disp('Wrong solver, choose another one in config.m');
    return
end

trackNameFile = 'FSG.mat'; %track name
load(trackNameFile);

track = Track(cones_blue, cones_yellow);

simulator = Simulator(config,parameters.car,parameters.tire);

mpc.setTrack(track);

trackCenter = mpc.getTrack().getPath();

trackPath = mpc.getTrack().getPath();
trackLength = mpc.getTrack().getLength();

% initial point
point0 = 1;

phi0 = atan2(trackPath.y(point0+1) - trackPath.y(point0),trackPath.x(point0+1) - trackPath.x(point0));
x0 = [trackPath.x(point0);trackPath.y(point0);phi0;0;0;0;0;0;0;0;0];

mpc.initMPC();

for i = 1:parameters.config.nSim
%for i = 1:10
    mpcSol = mpc.runMPC(x0);
    x0 = simulator.simTimeStep(x0,mpcSol.u0,parameters.config.ts);
    log(i) = mpcSol;
    disp("Iteraton:");
    disp(i);
    if mpcSol.solverStatus ~= 0
        error('solver returned status %d in closed loop iteration %d. Exiting.', mpcSol.solverStatus);
    end
end

addpath(genpath('plot'));

%% Plot car positions on all horizons
plotObj = Plot(config,parameters,track,log);
plotObj.carPositions();

%% Plot steeringAngle, sideSlipAngle, kinematicSideSlipAngle, frontSlipAngle, rearSlipAngle on all horizons
plotObj = Plot(config,parameters,track,log);
plotObj.angles();

%% Plot bounded states on all horizons
plotObj = Plot(config,parameters,track,log);
plotObj.boundedStates();

%% Plot bounded inputs on all horizons
plotObj = Plot(config,parameters,track,log);
plotObj.boundedInputs();

%% Plot race
plotObj = Plot(config,parameters,track,log);
plotObj.race();
%% Plot ax, ay
plotObj = Plot(config,parameters,track,log);
plotObj.accelerationsDistribution();
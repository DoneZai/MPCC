%test constraints

parameters = Parameters();
ts = 0.05;

%preconditions

[NX,NU,NB,NPC,NS,N,NSpline,siIndex] = config();
constraints = Constraints(ts, parameters.mpcModel, parameters.car, parameters.car);

%% Test 1

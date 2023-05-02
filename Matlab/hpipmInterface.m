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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [returnFlag, optimalSolution] = hpipmInterface(config,stages,x0)


% check that env.sh has been run
env_run = getenv('ENV_RUN');
if (~strcmp(env_run, 'true'))
	disp('ERROR: env.sh has not been sourced! Before using the HPIPM solver, run in the shell:');
	disp('source env.sh');
	disp('and then launch matlab or octave from the same shell.');
	disp('(tested on linux, untested on windows as of now).');
	return;
end


nx = config.NX; % number of state variables
nu = config.NU; % number of input variables
nbx = nx; % number of box constraints on state
nbu = nu; % number of box constraints on input
ns = config.NS; % number of soft constraints
npc = config.NPC; % number of polytopic constraints

N = config.N; % iters (horizon length)

%import hpipm_matlab.*

% dims
dims = hpipm_ocp_qp_dim(N);

dims.set('nx', 0, 0); % states number for 0 iter
dims.set('nx', nx, 1, N); % states number for 1 to N iters
dims.set('nu', nu, 0, N-1); % inputs number for 0 to N-1 iters
dims.set('nu', 0, N); % inputs number for N iter
dims.set('nbx', 0, 0); % box states constraints number for 0 iter  
dims.set('nbx', nbx, 1, N); % box states constraints number for 1 to N iters
dims.set('nbu', nbu, 0, N-1); % box inputs constraints number fot 1 to N-1 iters
dims.set('nbu', 0, N); % box inputs constraints number for N iter
dims.set('ng', 0, 0); % polytopic constraints number for 0 iter
dims.set('ng', npc, 1, N); % polytopic constraints number for 1 to N-1 iters
dims.set('ns', 0, 0); % soft constraints number for 0 iter
dims.set('ns', ns, 1, N); % soft constraints number for 1 to N-1 iters

% qp
qp = hpipm_ocp_qp(dims);
%% Dynamics
b0 = stages(1).linModel.a * stateToVector(x0) + stages(1).linModel.g;
for i = 0:N-1
    if i == 0
        qp.set('B', stages(i+1).linModel.b, i);
        qp.set('b', b0, i);
    else
        qp.set('A', stages(i+1).linModel.a, i); 
        qp.set('B', stages(i+1).linModel.b, i); 
        qp.set('b', stages(i+1).linModel.g, i);
    end
end

%% Cost
qp.set('Q', stages(1).costMat.Q, 0);
qp.set('R', stages(1).costMat.R, 0);
qp.set('S', stages(1).costMat.S, 0);
qp.set('q', stages(1).costMat.q, 0);
qp.set('r', stages(1).costMat.r, 0);
for i = 1:N
    qp.set('Q', stages(i+1).costMat.Q, i);
    qp.set('R', stages(i+1).costMat.R, i);
    qp.set('S', stages(i+1).costMat.S, i);
    qp.set('q', stages(i+1).costMat.q, i);
    qp.set('r', stages(i+1).costMat.r, i);
    qp.set('Zl', stages(i+1).costMat.Z, i);
    qp.set('Zu', stages(i+1).costMat.Z, i);
    qp.set('zl', stages(i+1).costMat.z, i);
    qp.set('zu', stages(i+1).costMat.z, i);
end

%% Polytopic Constraints
for i = 1:N
    qp.set('C', stages(i+1).constrainsMat.c, i);
    qp.set('D', stages(i+1).constrainsMat.d, i);
    qp.set('lg', stages(i+1).constrainsMat.dl, i);
    qp.set('ug', stages(i+1).constrainsMat.du, i);
end

%% Bounds
qp.set('Jbu', eye(4), 0, N-1);
qp.set('Jbx', eye(11), 1, N);
qp.set('lbu', stages(1).lBoundsU, 0);
qp.set('ubu', stages(1).uBoundsU, 0);
for i = 1:N-1
    qp.set('lbx', stages(i+1).lBoundsX,i);
    qp.set('ubx', stages(i+1).uBoundsX,i);
    qp.set('lbu', stages(i+1).lBoundsU,i);
    qp.set('ubu', stages(i+1).uBoundsU,i);
end
qp.set('lbx', stages(N+1).lBoundsX,N);
qp.set('ubx', stages(N+1).uBoundsX,N);

%% Soft Constraints
qp.set('Jsg', eye(npc,ns),1,N);
qp.set('Jsx', zeros(nbx,ns),1,N);
qp.set('Jsu', zeros(nbu,ns),1,N);
qp.set('lus', zeros(ns,1),1,N);
qp.set('lls', zeros(ns,1),1,N);

%%
qp_sol = hpipm_ocp_qp_sol(dims);


%% set up solver arg
%mode = 'speed_abs';
mode = 'speed';
%mode = 'balance';
%mode = 'robust';
% create and set default arg based on mode
arg = hpipm_ocp_qp_solver_arg(dims, mode);

%arg.set('mu0', 1e0);
arg.set('iter_max', 20);
%arg.set('tol_stat', 1e-6);
%arg.set('tol_eq', 1e-6);
%arg.set('tol_ineq', 1e-6);
%arg.set('tol_comp', 1e-5);
%arg.set('reg_prim', 1e-12);


% set up solver
solver = hpipm_ocp_qp_solver(dims, arg);

% solve qp
qptime = tic;
solver.solve(qp, qp_sol);
tmp_time = toc(qptime);

returnFlag = solver.get('status');

fprintf('solve time %e\n', tmp_time);

fprintf('HPIPM returned with flag %d ', returnFlag);

if returnFlag==0
    fprintf('-> QP solved\n')
else
    fprintf('-> Solver failed!\n')
end

time_ext = solver.get('time_ext');
iter = solver.get('iter');
resStat = solver.get('max_res_stat');
resEq = solver.get('max_res_eq');
resIneq = solver.get('max_res_ineq');
resComp = solver.get('max_res_comp');
stat = solver.get('stat');
fprintf('\nprint solver statistics\n');
fprintf('solve time of last run (measured in mex interface): %e [s]\n', time_ext);
fprintf('\nipm residuals max: res_g = %e, res_b = %e, res_d = %e, res_m = %e\n', resStat,resEq, resIneq, resComp);
fprintf('iter\talpha_aff\tmu_aff\t\tsigma\t\talpha_prim\talpha_dual\tmu\t\tres_stat\tres_eq\t\tres_ineq\tres_comp\n');
for ii=1:iter+1
    fprintf('%d\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\n', stat(ii,1), stat(ii,2), stat(ii,3), stat(ii,4), stat(ii,5), stat(ii,6), stat(ii,7), stat(ii,8), stat(ii,9), stat(ii,10), stat(ii,11));
end


optimalSolution(N+1) = OptVariables();
optimalSolution(1).xk.setZero();

for i = 1:N
    optimalSolution(i+1).xk = arrayToState(qp_sol.get('x',i));
end

for i = 0:N-1
    optimalSolution(i+1).uk = arrayToInput(qp_sol.get('u',i));
end

optimalSolution(N+1).uk.setZero();

end


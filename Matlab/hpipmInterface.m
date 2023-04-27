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


nx = config.NX;
nu = config.NU;
%ns = config.NS;
npc = config.NPC;

N = config.N;

qpTotal = tic;

%import hpipm_matlab.*

% dims
dims = hpipm_ocp_qp_dim(N);

dims.set('nx', 0, 0);
dims.set('nx', nx, 1, N);
dims.set('nu', nu, 0, N-1);
dims.set('nu', 0, N);
dims.set('nbx', 0, 0);
dims.set('nbx', nx, 1, N-1);
dims.set('nbu', nu, 0, N-1);
dims.set('nbu', 0, N);
dims.set('ng', 0, 0);
dims.set('ng', npc, 1, N-1);
%dims.set('nsbx', 0, 0, N-1);
%dims.set('nsbu', 0, 0, N-1)
%dims.set('nsg', 0, 0);
%dims.set('nsg', ns, 1, N-1);

%dims.print_C_struct();


% qp
qp = hpipm_ocp_qp(dims);
%% Dynamics
%x0 = blkdiag(MPC_vars.Tx,MPC_vars.Tu)*[stage(1).x0;stage(1).u0];
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
for i = 0:N-1
    qp.set('Q', stages(i+1).costMat.Q, i);
    qp.set('R', stages(i+1).costMat.R, i);
    qp.set('S', stages(i+1).costMat.S, i);
    qp.set('q', stages(i+1).costMat.q, i);
    qp.set('r', stages(i+1).costMat.r, i);
    %if stages(i+1).ns ~= 0.0
    %    qp.set('Zl', stages(i+1).costMat.Z, i);
    %    qp.set('Zu', stages(i+1).costMat.Z, i);
    %    qp.set('zl', stages(i+1).costMat.z, i);
    %    qp.set('zu', stages(i+1).costMat.z, i);
    %end
end
%% Polytopic Constraints
for i = 1:N-1
    qp.set('C', stages(i+1).constrainsMat.c, i);
    qp.set('D', stages(i+1).constrainsMat.d, i);
    qp.set('lg', stages(i+1).constrainsMat.dl, i);
    qp.set('ug', stages(i+1).constrainsMat.du, i);
end

%% Bounds
qp.set('Jbu', eye(4), 0, N-1);
qp.set('Jbx', eye(11), 1, N-1);
qp.set('lbu', stages(1).lBoundsU, 0);
qp.set('ubu', stages(1).uBoundsU, 0);
for i = 1:N-1
    qp.set('lbx', stages(i+1).lBoundsX,i);
    qp.set('ubx', stages(i+1).uBoundsX,i);
    qp.set('lbu', stages(i+1).lBoundsU,i);
    qp.set('ubu', stages(i+1).uBoundsU,i);
end

%% Soft Constraints
%for i = 0:N-1
%    if stages(i+1).ns ~= 0
%        
%    end
%end

    
%qp.print_C_struct();


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
arg.set('iter_max', 60);
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
%     qp_sol.print_C_struct()
else
    fprintf('-> Solver failed!\n')
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


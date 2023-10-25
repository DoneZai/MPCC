classdef Mpcc < handle
    %MPC Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        d_config
        
        d_ts
        d_validInitialGuess
    
        d_stages
        d_initialGuess
        d_optimalSolution
    
        d_nSqp
        d_sqpMixing
        d_nNonSolves
        d_nNoSolvesSqp
        d_nReset
    
        d_model
        d_cost
        d_constraints
        d_track
        d_bounds
        d_normalization
        d_mpcModel
        d_car
        %d_solverInterface
    end
    
    methods (Access = public)
        function obj = Mpcc(config,parameters)
            obj.d_config = config;

            obj.d_ts = parameters.config.ts;
            obj.d_validInitialGuess = false;
            
            stages(1:obj.d_config.N + 1) = Stage(obj.d_config);
            initialGuess(1:obj.d_config.N + 1) = OptVariables();
            optimalSolution(1:obj.d_config.N + 1) = OptVariables();

            for i = 1:obj.d_config.N + 1
                stages(i) = Stage(obj.d_config);
                initialGuess(i) = OptVariables();
                optimalSolution(i) = OptVariables();
            end

            obj.d_stages = stages;
            obj.d_initialGuess = initialGuess;
            obj.d_optimalSolution = optimalSolution;
           
            obj.d_nSqp = parameters.config.nSqp;
            obj.d_sqpMixing = parameters.config.sqpMixing;
            obj.d_nNonSolves = 0.0;
            obj.d_nNoSolvesSqp = 0.0;
            obj.d_nReset = parameters.config.nReset;
            
            obj.d_model = LinModel(obj.d_config,obj.d_ts,parameters.car,parameters.tire);
            obj.d_cost = Cost(obj.d_config,parameters.costs, parameters.car);
            obj.d_constraints = Constraints(obj.d_config,obj.d_ts,parameters.mpcModel,parameters.car,parameters.tire);
            obj.d_track = ArcLengthSpline(obj.d_config,parameters.mpcModel);
            obj.d_bounds = Bounds(parameters.bounds);
            obj.d_normalization = parameters.normalization;
            obj.d_mpcModel = parameters.mpcModel;
            obj.d_car = parameters.car;
            %obj.d_solverInterface = HpipmInterface(obj.d_config);
        end

        function mpcReturn = runMPC(obj,xVec)
            %profile on;
            solverStatus = -1;
            x0 = vectorToState(xVec);
            x0.s = obj.d_track.projectOnSpline(x0);
            x0.unwrap(obj.d_track.getLength());
            if obj.d_validInitialGuess
              obj.updateInitialGuess(x0);
            else
              obj.generateNewInitialGuess(x0);
            end
            % TODO: this is one approach to handle solver errors, works well in simulatio
            obj.d_nNoSolvesSqp = 0;
            for i = 1:obj.d_nSqp
              obj.setMPCProblem();
              x0Normalized = vectorToState(...
                obj.d_normalization.tXInv * (stateToVector(x0) - 1.0 * stateToVector(x0)));
              %profile viewer;
              [solverStatus, obj.d_optimalSolution] = hpipmInterface(obj.d_config,obj.d_stages, x0Normalized);
              obj.d_optimalSolution = obj.deNormalizeSolution(obj.d_optimalSolution);
              if solverStatus ~= 0
                  obj.d_nNoSolvesSqp = obj.d_nNoSolvesSqp + 1;
              end
              if solverStatus <= 1
                  obj.d_initialGuess = obj.sqpSolutionUpdate(obj.d_initialGuess, obj.d_optimalSolution);
              end
            end
            maxError = max(obj.d_nSqp - 1, 1);
            if obj.d_nNoSolvesSqp >= maxError
              obj.d_nNonSolves = obj.d_nNonSolves + 1;
            else
              obj.d_nNonSolves = 0;
            end
            if obj.d_nNonSolves >= obj.d_nReset
              obj.d_validInitialGuess = false;
            end
            mpcReturn = MpcReturn(stateToVector(obj.d_initialGuess(2).xk), inputToVector(obj.d_initialGuess(1).uk), obj.getInitialGuessMatrix(),solverStatus);
        end

        function setTrack(obj,track)
            obj.d_track.gen2DSpline(track.x,track.y);
        end
    
        function track = getTrack(obj)
            track = obj.d_track;
        end
    end

    methods (Access = private)

        function matrix = getInitialGuessMatrix(obj)
            matrix = zeros(11,length(obj.d_initialGuess));
            for i = 1:length(obj.d_initialGuess)
                matrix(:,i) =  stateToVector(obj.d_initialGuess(i).xk);
            end
        end

        function setMPCProblem(obj)
            for i = 1:obj.d_config.N
                obj.setStage(obj.d_initialGuess(i).xk, obj.d_initialGuess(i).uk, obj.d_initialGuess(i + 1).xk, i);
            end
            xk1Nz = vectorToState(obj.d_model.ode4(obj.d_initialGuess(obj.d_config.N+1).xk,obj.d_initialGuess(obj.d_config.N+1).uk,obj.d_ts));
            obj.setStage(obj.d_initialGuess(obj.d_config.N+1).xk, obj.d_initialGuess(obj.d_config.N+1).uk, xk1Nz, obj.d_config.N+1);
        end

        function setStage(obj,xk,uk,xk1,timeStep)
              obj.d_stages(timeStep).nx = obj.d_config.NX;
              obj.d_stages(timeStep).nu = obj.d_config.NU;
            
              if timeStep == 1
                obj.d_stages(timeStep).ng = 0;
                obj.d_stages(timeStep).ns = 0;
              else
                obj.d_stages(timeStep).ng = obj.d_config.NPC;
                obj.d_stages(timeStep).ns = obj.d_config.NS;
              end
            
              vxZero = obj.d_mpcModel.initialVelocity;
            
              xkNz = xk;
              xkNz.vxNonZero(vxZero);
            
              xk1Nz = xk1;
              xk1Nz.vxNonZero(vxZero);
            
              obj.d_stages(timeStep).costMat = obj.normalizeCost(obj.d_cost.getCost(obj.d_track, xkNz, uk, timeStep));
              obj.d_stages(timeStep).linModel = obj.normalizeDynamics(obj.d_model.getLinModel(xkNz, uk, xk1Nz));
              obj.d_stages(timeStep).constrainsMat =...
                obj.normalizeCon(obj.d_constraints.getConstraints(obj.d_track, xkNz));
            
              obj.d_stages(timeStep).lBoundsX =...
                obj.d_normalization.tXInv * obj.d_bounds.getBoundsLX(xkNz);
              obj.d_stages(timeStep).uBoundsX =...
                obj.d_normalization.tXInv * obj.d_bounds.getBoundsUX(xkNz);
              obj.d_stages(timeStep).lBoundsU = obj.d_normalization.tUInv * obj.d_bounds.getBoundsLU(uk);
              obj.d_stages(timeStep).uBoundsU = obj.d_normalization.tUInv * obj.d_bounds.getBoundsUU(uk);
              obj.d_stages(timeStep).lBoundsS = obj.d_normalization.tSInv * obj.d_bounds.getBoundsLS();
              obj.d_stages(timeStep).uBoundsS = obj.d_normalization.tSInv * obj.d_bounds.getBoundsUS();
            
              obj.d_stages(timeStep).lBoundsX(obj.d_config.siIndex.s) =...
                obj.d_normalization.tXInv(obj.d_config.siIndex.s,obj.d_config.siIndex.s) *...
                (-obj.d_mpcModel.sTrustRegion);  %*d_initialGuess[timeStep].xk.vs;
              obj.d_stages(timeStep).uBoundsX(obj.d_config.siIndex.s) =...
                obj.d_normalization.tXInv(obj.d_config.siIndex.s,obj.d_config.siIndex.s) *...
                (obj.d_mpcModel.sTrustRegion);  %*d_initialGuess[timeStep].xk.vs;
        end


        function costMat = normalizeCost(obj,costMat)
              Q = obj.d_normalization.tX * costMat.Q * obj.d_normalization.tX;
              R = obj.d_normalization.tU * costMat.R * obj.d_normalization.tU;
              q = obj.d_normalization.tX * costMat.q;
              r = obj.d_normalization.tU * costMat.r;
              Z = obj.d_normalization.tS * costMat.Z * obj.d_normalization.tS;
              z = obj.d_normalization.tS * costMat.z;
              costMat = CostMatrix(Q, R, zeros(obj.d_config.NX,obj.d_config.NU), q, r, Z, z);
        end

        function linModelMat = normalizeDynamics(obj,linModel)
              a =...
              obj.d_normalization.tXInv * linModel.a * obj.d_normalization.tX;
              b =...
              obj.d_normalization.tXInv * linModel.b * obj.d_normalization.tU;
              g = obj.d_normalization.tXInv * linModel.g;
              linModelMat = LinModelMatrix(a,b,g);
        end


        function mat = normalizeCon(obj,conMat)
              c = conMat.c * obj.d_normalization.tX;
              d = conMat.d * obj.d_normalization.tU;
              dl = conMat.dl;
              du = conMat.du;
              mat = ConstraintsMatrix(c,d,dl,du);
        end

        function denormalizedSolution = deNormalizeSolution(obj,solution)
            denormalizedSolution(obj.d_config.N+1) = OptVariables();
            for i = 1:obj.d_config.N+1
              updatedXVec = obj.d_normalization.tX * stateToVector(solution(i).xk);
              updatedUVec = obj.d_normalization.tU * inputToVector(solution(i).uk);
              denormalizedSolution(i).xk = vectorToState(updatedXVec);
              denormalizedSolution(i).uk = vectorToInput(updatedUVec);
            end
        end

        function updateInitialGuess(obj,x0)
            for i = 2:obj.d_config.N 
                obj.d_initialGuess(i - 1) = obj.d_initialGuess(i);
            end
            obj.d_initialGuess(1).xk = x0;
            obj.d_initialGuess(1).uk.setZero();
            obj.d_initialGuess(obj.d_config.N).xk = obj.d_initialGuess(obj.d_config.N - 1).xk; 
            obj.d_initialGuess(obj.d_config.N).uk.setZero();  % = d_initialGuess[N-2].uk;
            %[~,inivt]=ode45(@(t,inivt)obj.d_model.getF(t,obj.d_initialGuess(obj.d_config.N).xk,obj.d_initialGuess(obj.d_config.N).uk), ...
            %    [0 obj.d_ts],obj.d_initialGuess(obj.d_config.N).xk);
            %obj.d_initialGuess(obj.d_config.N+1).xk = inivt(end,:);
            obj.d_initialGuess(obj.d_config.N+1).xk = vectorToState(obj.d_model.ode4(obj.d_initialGuess(obj.d_config.N).xk,obj.d_initialGuess(obj.d_config.N).uk,obj.d_ts));
            obj.d_initialGuess(obj.d_config.N+1).uk.setZero();
            obj.unwrapInitialGuess();
        end

        function generateNewInitialGuess(obj,x0)
            obj.d_initialGuess(1).xk = x0;
            obj.d_initialGuess(1).uk.setZero();
            for i = 2:obj.d_config.N+1
              obj.d_initialGuess(i).xk.setZero();
              obj.d_initialGuess(i).uk.setZero();
              obj.d_initialGuess(i).xk.s =...
                obj.d_initialGuess(i - 1).xk.s + obj.d_ts * obj.d_mpcModel.initialVelocity;
              trackPosI = obj.d_track.getPosition(obj.d_initialGuess(i).xk.s);
              trackdPosI = obj.d_track.getDerivative(obj.d_initialGuess(i).xk.s);
              obj.d_initialGuess(i).xk.x = trackPosI(1);
              obj.d_initialGuess(i).xk.y = trackPosI(2);
              obj.d_initialGuess(i).xk.yaw = atan2(trackdPosI(2), trackdPosI(1));
              obj.d_initialGuess(i).xk.vx = obj.d_mpcModel.initialVelocity;
              obj.d_initialGuess(i).xk.vs = obj.d_mpcModel.initialVelocity;
            end
            obj.unwrapInitialGuess();
            obj.d_validInitialGuess = true;
        end

        function unwrapInitialGuess(obj)
            L = obj.d_track.getLength();
            for i = 2:obj.d_config.N+1
              if (obj.d_initialGuess(i).xk.yaw - obj.d_initialGuess(i - 1).xk.yaw) < -pi
                obj.d_initialGuess(i).xk.yaw = obj.d_initialGuess(i).xk.yaw + 2.0 * pi;
              end
              if (obj.d_initialGuess(i).xk.yaw - obj.d_initialGuess(i - 1).xk.yaw) > pi
                obj.d_initialGuess(i).xk.yaw = obj.d_initialGuess(i).xk.yaw - 2.0 * pi;
              end
              if (obj.d_initialGuess(i).xk.s - obj.d_initialGuess(i - 1).xk.s) > L / 2.0
                obj.d_initialGuess(i).xk.s = obj.d_initialGuess(i).xk.s - L;
              end
           end
        end

        function updatedSolution = sqpSolutionUpdate(obj,lastSolution,currentSolution)
            % TODO use line search and merit function
            updatedSolution(obj.d_config.N+1) = OptVariables();
            for i = 1:obj.d_config.N+1
              updatedXVec =...
                obj.d_sqpMixing * (stateToVector(currentSolution(i).xk) + stateToVector(lastSolution(i).xk)) +...
                (1.0 - obj.d_sqpMixing) * stateToVector(lastSolution(i).xk);
              updatedUVec =...
                obj.d_sqpMixing * (inputToVector(currentSolution(i).uk) + inputToVector(lastSolution(i).uk)) +...
                (1.0 - obj.d_sqpMixing) * inputToVector(lastSolution(i).uk);
              updatedSolution(i).xk = vectorToState(updatedXVec);
              updatedSolution(i).uk = vectorToInput(updatedUVec);
            end
        end
    end
end


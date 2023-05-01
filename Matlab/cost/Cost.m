classdef Cost
    %COST Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        d_config

        d_costParameters
        d_carParameters
    end
    
    methods (Access = public)
        function obj = Cost(config,costParameters,carParameters)
            obj.d_config = config;
            obj.d_costParameters = costParameters;
            obj.d_carParameters = carParameters;
        end

        function cost = getCost(obj,track,state,u,k)
            % generate quadratic cost function
            contouringCost = obj.getContouringCost(track, state, k);
            headingCost = obj.getHeadingCost(track, state);
            inputCost = obj.getInputCost();

            if (obj.d_costParameters.betaKinCost == 1)
              betaCost = obj.getBetaKinCost(state);
            else
              betaCost = obj.getBetaCost(state);
            end

            softConCost = obj.getSoftConstraintCost();

            QNotSym = contouringCost.Q + headingCost.Q + inputCost.Q + betaCost.Q;
            % QMPC Q_reg = 1e-9 * QMPC::Identity();
            QFull = 0.5 * (QNotSym' + QNotSym);
            RFull = contouringCost.R + headingCost.R + inputCost.R + betaCost.R;
            qFull = contouringCost.q + headingCost.q + inputCost.q + betaCost.q;
            rFull = contouringCost.r + headingCost.r + inputCost.r + betaCost.r;
            % TODO do this properly directly in the different functions computing the cost
            Q = QFull;
            R = RFull;
            q = qFull + ((stateToVector(state))' * QFull)';
            r = rFull + ((inputToVector(u))' * RFull)';
            Z = 2.0 * softConCost.Z;
            z = softConCost.z;
            % std::cout << Z(0,0) << ' ' << Z(1,1) << std::endl;
            cost = CostMatrix(Q, R, zeros(11,4), q, r, Z, z);
        end
        
       
    end

    methods (Access = private)
        function point = getRefPoint(obj,track,state)
            % compute all the geometry information of the track at a given arc length
            s = state.s;
            % X-Y postion of the reference at s
            posRef = track.getPosition(s);
            xRef = posRef(1);
            yRef = posRef(2);
            % reference path derivatives
            dposRef = track.getDerivative(s);
            dxRef = dposRef(1);
            dyRef = dposRef(2);
            % angle of the reference path
            thetaRef = atan2(dyRef, dxRef);
            % second order derivatives
            ddposRef = track.getSecondDerivative(s);
            ddxRef = ddposRef(1);
            ddyRef = ddposRef(2);
            % curvature
            dthetaRefNom = (dxRef * ddyRef - dyRef * ddxRef);
            dthetaRefDenom = (dxRef * dxRef + dyRef * dyRef);
            % if(std::fabs(dthetaRefNom) < 1e-7)
            %     dthetaRefNom = 0;
            % if(std::fabs(dthetaRefDenom) < 1e-7)
            %     dthetaRefDenom = 1e-7;
            dthetaRef = dthetaRefNom / dthetaRefDenom;
            point = TrackPoint(xRef, yRef, dxRef, dyRef, thetaRef, dthetaRef);
        end

        function errorInfo = getErrorInfo(obj,track,state)
            % compute error between reference and X-Y position of the car
            x = state.x;
            y = state.y;
            trackPoint = obj.getRefPoint(track, state);
            % contouring  error
            contouringErrorMat = zeros(1,2);
            contouringErrorMat(1) = -sin(trackPoint.thetaRef) * (trackPoint.xRef - x)...
                                    + cos(trackPoint.thetaRef) * (trackPoint.yRef - y);
            % lag error
            contouringErrorMat(2) = cos(trackPoint.thetaRef) * (trackPoint.xRef - x)...
                                    + sin(trackPoint.thetaRef) * (trackPoint.yRef - y);
            % partial derivatives of the lag and contouring error with respect to s
            dContouringError =...
              -trackPoint.dthetaRef * cos(trackPoint.thetaRef) * (trackPoint.xRef - x)...
              - trackPoint.dthetaRef * sin(trackPoint.thetaRef) * (trackPoint.yRef - y)...
              - trackPoint.dxRef * sin(trackPoint.thetaRef)...
              + trackPoint.dyRef * cos(trackPoint.thetaRef);
            dLagError =...
              -trackPoint.dthetaRef * sin(trackPoint.thetaRef) * (trackPoint.xRef - x)...
              + trackPoint.dthetaRef * cos(trackPoint.thetaRef) * (trackPoint.yRef - y)...
              + trackPoint.dxRef * cos(trackPoint.thetaRef) +...
              trackPoint.dyRef * sin(trackPoint.thetaRef);
            dContouringErrorMat = zeros(2,obj.d_config.NX);
            % compute all remaining partial derivatives and store them in dError
            dContouringErrorMat(1, obj.d_config.siIndex.x) = sin(trackPoint.thetaRef);
            dContouringErrorMat(1, obj.d_config.siIndex.y) = -cos(trackPoint.thetaRef);
            dContouringErrorMat(1, obj.d_config.siIndex.s) = dContouringError;
            dContouringErrorMat(2, obj.d_config.siIndex.x) = -cos(trackPoint.thetaRef);
            dContouringErrorMat(2, obj.d_config.siIndex.y) = -sin(trackPoint.thetaRef);
            dContouringErrorMat(2, obj.d_config.siIndex.s) = dLagError;
            errorInfo = ErrorInfo(contouringErrorMat, dContouringErrorMat);
        end

        function contouringCost = getContouringCost(obj,track,state,k)
            % compute state cost, formed by contouring error cost + cost on "real" inputs
            % compute reference information
            % const StateVector x_vec = stateToVector(x);
            % compute error and jacobean of error
            errorInfo = obj.getErrorInfo(track, state);
            % contouring cost matrix
            ContouringCost = zeros(2,1);
            if k < obj.d_config.N
                ContouringCost(1) = obj.d_costParameters.qC;
            else
                ContouringCost(1) = obj.d_costParameters.qCNmult * obj.d_costParameters.qC;
            end
            ContouringCost(2) = obj.d_costParameters.qL;
            % contouring and lag error part
            dContouringError = errorInfo.dError(1,:);

            contouringErrorMatZero =...
              errorInfo.error(1) - dContouringError * stateToVector(state);

            dLagError = errorInfo.dError(2,:);

            lagErrorZero = errorInfo.error(2) - dLagError * stateToVector(state);
            QContouringCost = ContouringCost(1) * (dContouringError' * dContouringError) +...
                              ContouringCost(2) * (dLagError' * dLagError);
            % regularization cost on yaw rate
            if k < obj.d_config.N
                QContouringCost(obj.d_config.siIndex.r, obj.d_config.siIndex.r) = obj.d_costParameters.qR;
            else
                QContouringCost(obj.d_config.siIndex.r, obj.d_config.siIndex.r) = obj.d_costParameters.qRNmult * obj.d_costParameters.qR;
            end
            QContouringCost = 2.0 * QContouringCost;
            qContouringCost =...
              ContouringCost(1) * 2.0 * contouringErrorMatZero * dContouringError' +...
              ContouringCost(2) * 2.0 * lagErrorZero * dLagError';
            % progress maximization part
            qContouringCost(obj.d_config.siIndex.vs) = -obj.d_costParameters.qVs;
            % solver interface expects 0.5 x^T Q x + q^T x
        
            contouringCost = CostMatrix(QContouringCost, zeros(4,4), zeros(11,4), qContouringCost, zeros(4,1), zeros(2,2), zeros(2,1));
        end

        function headingCost = getHeadingCost(obj,track,state)
            % get heading of the track
            dposRef = track.getDerivative(state.s);
            dxRef = dposRef(1);
            dyRef = dposRef(2);
            % angle of the reference path
            thetaRef = atan2(dyRef, dxRef);
            thetaRef = thetaRef + 2.0 * pi * round((state.yaw - thetaRef) / (2.0 * pi));
            QHeadingCost = zeros(11,11);
            QHeadingCost(obj.d_config.siIndex.yaw, obj.d_config.siIndex.yaw) = 2.0 * obj.d_costParameters.qMu;
            qHeadingCost = zeros(11,1);
            qHeadingCost(obj.d_config.siIndex.yaw) = -2.0 * obj.d_costParameters.qMu * thetaRef;
            headingCost = CostMatrix(QHeadingCost, zeros(4,4), zeros(11,4), qHeadingCost, zeros(4,1), zeros(2,2), zeros(2,1));
        end

        function inputCost = getInputCost(obj)
            % input cost and rate of chagen of real inputs
            qInputCost = zeros(11,11);
            rInputCost = zeros(4,4);
            % cost of "real" inputs
            qInputCost(obj.d_config.siIndex.throttle, obj.d_config.siIndex.throttle) = obj.d_costParameters.rThrottle;
            qInputCost(obj.d_config.siIndex.steeringAngle, obj.d_config.siIndex.steeringAngle) = obj.d_costParameters.rSteeringAngle;
            qInputCost(obj.d_config.siIndex.brakes, obj.d_config.siIndex.brakes) = obj.d_costParameters.rBrakes;
            qInputCost(obj.d_config.siIndex.vs, obj.d_config.siIndex.vs) = obj.d_costParameters.rVs;
            % quadratic part
            rInputCost(obj.d_config.siIndex.dThrottle, obj.d_config.siIndex.dThrottle) = obj.d_costParameters.rdThrottle;
            rInputCost(obj.d_config.siIndex.dSteeringAngle, obj.d_config.siIndex.dSteeringAngle) = obj.d_costParameters.rdSteeringAngle;
            rInputCost(obj.d_config.siIndex.dBrakes, obj.d_config.siIndex.dBrakes) = obj.d_costParameters.rdBrakes;
            rInputCost(obj.d_config.siIndex.dVs, obj.d_config.siIndex.dVs) = obj.d_costParameters.rdVs;
            % solver interface expects 0.5 u^T R u + r^T u
            qInputCost = 2.0 * qInputCost;
            rInputCost = 2.0 * rInputCost;
            inputCost = CostMatrix(qInputCost, rInputCost, zeros(11,4), zeros(11,1), zeros(4,1), zeros(2,2), zeros(2,1));
        end

        function betaCost = getBetaCost(obj,state)
            %    CostMatrix betaCost;
            vx = state.vx;
            vy = state.vy;
            % jacobian of beta
            dBeta = zeros(1,obj.d_config.NX);
            dBeta(obj.d_config.siIndex.vx) = -vy / (vx * vx + vy * vy);
            dBeta(obj.d_config.siIndex.vy) = vx / (vx * vx + vy * vy);
            % zero order term of beta approximation
            betaZero = atan(vy / vx) - dBeta * stateToVector(state);
            % QBeta = (qBeta*beta)^2 ~ x^T (qBeta*dBeta^T*dBeta) x + (qBeta*2*BetaZero*qBeta)^ x + const
            QBeta = 2.0 * obj.d_costParameters.qBeta * (dBeta' * dBeta);
            qBeta = obj.d_costParameters.qBeta * 2.0 * betaZero * dBeta';
            betaCost =  CostMatrix(QBeta, zeros(4,4), zeros(11,4), qBeta, zeros(4,1), zeros(2,2), zeros(2,1));
        end

        function betaKinCost = getBetaKinCost(obj,state)
            relCenter = obj.d_carParameters.lr / (obj.d_carParameters.lf + obj.d_carParameters.lr);
            %    CostMatrix betaCost;
            vx = state.vx;
            vy = state.vy;
            steeringAngle = state.steeringAngle;
            % jacobian of beta
            dBeta = zeros(1,obj.d_config.NX);
            dBeta(obj.d_config.siIndex.vx) = vy / (vx * vx + vy * vy);
            dBeta(obj.d_config.siIndex.vy) = -vx / (vx * vx + vy * vy);
            dBeta(obj.d_config.siIndex.steeringAngle) =...
              (relCenter * (1.0 / cos(steeringAngle)) * (1.0 / cos(steeringAngle))) /...
              (relCenter * relCenter * tan(steeringAngle) * tan(steeringAngle) + 1.0);
            % zero order term of beta approximation
            betaZero = atan(tan(steeringAngle) * relCenter) - atan(vy / vx) -...
                                    dBeta * stateToVector(state);
            % QBeta = (qBeta*beta)^2 ~ x^T (qBeta*dBeta^T*dBeta) x + (qBeta*2*BetaZero*qBeta)^ x + const
            QBeta = 2.0 * obj.d_costParameters.qBeta * (dBeta' * dBeta);
            qBeta = obj.d_costParameters.qBeta * 2.0 * betaZero * dBeta';
            betaKinCost = CostMatrix(QBeta, zeros(4,4), zeros(11,4), qBeta, zeros(4,1), zeros(2,2), zeros(2,1));
        end

        function softConstraintsCost = getSoftConstraintCost(obj)
            % input cost and rate of chagen of real inputs
            ZCost = eye(2);
            zCost = ones(2,1);
            % cost of "real" inputs
            ZCost(obj.d_config.siIndex.conTrack, obj.d_config.siIndex.conTrack) = obj.d_costParameters.scQuadTrack;
            ZCost(obj.d_config.siIndex.conAlpha, obj.d_config.siIndex.conAlpha) = obj.d_costParameters.scQuadAlpha;
            zCost(obj.d_config.siIndex.conTrack) = obj.d_costParameters.scLinTrack;
            zCost(obj.d_config.siIndex.conAlpha) = obj.d_costParameters.scLinAlpha;
            softConstraintsCost = CostMatrix(zeros(11,11), zeros(4,4), zeros(11,4), zeros(11,1), zeros(4,1), ZCost, zCost);
        end

    end
end


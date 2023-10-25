classdef Constraints
    %CONSTRAINTS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        d_config;
        d_model
        d_parameters
    end
    
    methods (Access = public)
        function obj = Constraints(config,ts,modelParameters,carParameters,tireParameters)
            obj.d_config = config;
            obj.d_model = LinModel(obj.d_config,ts,carParameters,tireParameters);
            obj.d_parameters = modelParameters;
        end

         function constraints = getConstraints(obj,track,x)
            % compute all the polytrophic state constraints
            % compute the three constraints
            trackConstraints = obj.getTrackConstraints(track, x);
            alphaConstraintsFront = obj.getAlphaConstraintFront(x);

            cConstrainsMatrix = zeros(obj.d_config.NPC,obj.d_config.NX);
            dlConstrainsMatrix = zeros(obj.d_config.NPC,1);
            duConstrainsMatrix = zeros(obj.d_config.NPC,1);

            cConstrainsMatrix(obj.d_config.siIndex.conTrack,:) = trackConstraints.cI;
            dlConstrainsMatrix(obj.d_config.siIndex.conTrack) = trackConstraints.dlI;
            duConstrainsMatrix(obj.d_config.siIndex.conTrack) = trackConstraints.duI;

            cConstrainsMatrix(obj.d_config.siIndex.conAlpha,:) = alphaConstraintsFront.cI;
            dlConstrainsMatrix(obj.d_config.siIndex.conAlpha) = alphaConstraintsFront.dlI;
            duConstrainsMatrix(obj.d_config.siIndex.conAlpha) = alphaConstraintsFront.duI;

            % TODO consider the zero order term directly in the functions constructing the constraints
            dlConstrainsMatrix = dlConstrainsMatrix - cConstrainsMatrix * stateToVector(x);
            duConstrainsMatrix = duConstrainsMatrix - cConstrainsMatrix * stateToVector(x);
            constraints = ConstraintsMatrix(cConstrainsMatrix, zeros(obj.d_config.NPC,obj.d_config.NU), dlConstrainsMatrix, duConstrainsMatrix);
        end
    end

    methods (Access = private)
        function trackConstraints = getTrackConstraints(obj,track,x)
            % given arc length s and the track -> compute linearized track constraints
            s = x.s;
            % X-Y point of the center line
            posCenter = track.getPosition(s);
            dCenter = track.getDerivative(s);
            % Tangent of center line at s
            tanCenter = [-dCenter(2), dCenter(1)];
            % inner and outer track boundary given left and right width of track
            % TODO make R_out and R_in dependent on s
            posOuter = posCenter + obj.d_parameters.rOut * tanCenter;
            posInner = posCenter - obj.d_parameters.rIn * tanCenter;
            % Define track Jacobian as Perpendicular vector
            CTrackConstraint = zeros(1,obj.d_config.NX);
            CTrackConstraint(1, 1) = tanCenter(1);
            CTrackConstraint(1, 2) = tanCenter(2);
            % Compute bounds
            trackConstraintLower = tanCenter(1) * posInner(1) + tanCenter(2) * posInner(2);
            trackConstraintUpper = tanCenter(1) * posOuter(1) + tanCenter(2) * posOuter(2);
            trackConstraints = OneDConstraint(CTrackConstraint,trackConstraintLower,trackConstraintUpper);
        end

        function alphaConstraint = getAlphaConstraintFront(obj,x)
            % compute linearized slip angle constraints
            % -alpha_max <= alphaF <= alpha_max
            xVec = stateToVector(x);
            alphaF = obj.d_model.getSaFront(x);
            % compute the jacobean of alphaF
            cAlphaConstraint = obj.getAlphaConstraintFrontJac(x);
            % compute the bounds given the Tylor series expansion
            alphaConstraintLower = -obj.d_parameters.maxAlpha - alphaF + cAlphaConstraint * xVec;
            alphaConstraintUpper = obj.d_parameters.maxAlpha - alphaF + cAlphaConstraint * xVec;
            alphaConstraint = OneDConstraint(cAlphaConstraint, alphaConstraintLower, alphaConstraintUpper);
        end

        function jacAlphaCon = getAlphaConstraintFrontJac(obj,x)
            % const WheelSpeeds ws = d_model.getFrontWheelSpeeds(x);
            dSa = obj.d_model.getFrontSaDerivatives(x);
            % const UxDerivatives dUx = d_model.getFrontUxDerivatives(x);
            % const UyDerivatives dUy = d_model.getFrontUyDerivatives(x);
            jacAlphaCon = zeros(1,obj.d_config.NX);
            jacAlphaCon(obj.d_config.siIndex.vx) = dSa.dVx;
            jacAlphaCon(obj.d_config.siIndex.vy) = dSa.dVy;
            jacAlphaCon(obj.d_config.siIndex.r) = dSa.dR;
            jacAlphaCon(obj.d_config.siIndex.steeringAngle) = dSa.dSteeringAngle;
        end
    end
end


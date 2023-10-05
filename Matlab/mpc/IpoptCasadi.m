classdef IpoptCasadi
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        d_config
        d_parameters
        d_ts
        d_car
        d_tire
        d_track

        N
        NX
        NU
        NS

        X
        U
        P
        S
        f
        g
        objective

        Q
        R
        Z

        solver
        opts

        lbx
        ubx
        lbg
        ubg
    end
    
    methods (Access = public)
        function obj = IpoptCasadi(config,parameters)
            obj.d_config = config;
            obj.d_parameters = parameters;
            obj.d_ts = parameters.config.ts;

            obj.d_track = ArcLengthSpline(config,parameters.mpcModel); 
        end

        function initMPC()
            
        end

        function setTrack(obj,x,y)
            obj.d_track.gen2DSpline(x,y);
        end

        function obj = initSystemModel(obj)
            obj.N = obj.d_config.N;
            obj.NX = obj.d_config.NX;
            obj.NU = obj.d_config.NU;
            obj.NS = obj.d_config.NS;

            % States
            x = SX.sym('x');
            y = SX.sym('y');
            yaw = SX.sym('yaw');
            vx = SX.sym('vx');
            vy = SX.sym('vy');
            r = SX.sym('r');
            s = SX.sym('s');
            throttle = SX.sym('throttle');
            steeringAngle = SX.sym('steeringAngle');
            brakes = SX.sym('brakes');
            vs = SX.sym('vs');
            
            states = [x;y;yaw;vx;vy;r;s;throttle;steeringAngle;brakes;vs];
            
            % Controls
            dThrottle = SX.sym('dThrottle');
            dSteeringAngle = SX.sym('dSteeringAngle');
            dBrakes = SX.sym('dBrakes');
            dVs = SX.sym('dVs');

            controls = [dThrottle,dSteeringAngle,dBrakes];

            %Dynamic forces
            rDyn = obj.d_car.rDyn;
            
            cdrv = obj.d_car.cm1 * obj.d_car.gearRatio;
            cbf = obj.d_car.cbf;
            cbr = obj.d_car.cbr;

            m = obj.d_car.m;
            lf = obj.d_car.lf;
            lr = obj.d_car.lr;
            gAcc = obj.d_car.g;
            fzNominal = obj.d_car.fzNominal;

            % normal load on the one front wheel
            Ffz = lr * m * gAcc /...
                          (2.0 * (lf + lr));
            Dffz = (Ffz - fzNominal) / fzNominal;

            % normal load on the one rear wheel
            Frz = lf * m * gAcc /...
                          (2.0 * (lf + lr));
            Drfz = (Frz - fzNominal) / fzNominal;
            
            % rolling resistance of the two front wheels
            Ffrr = 2*obj.d_tire.QSY1*Ffz;

            % rolling resistance of the two rear wheels
            Frrr = 2*obj.d_tire.QSY1*Frz;

            % slip angle of the front wheel
            saf = atan2((vy + r * lf), vx) - steeringAngle;

            % slip angle of the rear wheel
            sar = atan2((vy - r * lr), vx);

            % longitudinal front force
            Ffx = (-cbf*brakes)/rDyn + Ffrr;
            
            % longitudinal rear force
            Frx = (-cbr*brakes + cdrv*throttle)/rDyn + Frrr;

            % latteral tire force Pacejka coefficients
            % front wheel coefficients
            Kfy = obj.d_tire.PKY1 * fzNominal *...
            sin(2.0 * atan2(Ffz, (obj.d_tire.PKY2 * fzNominal * obj.d_tire.LFZO))) * obj.d_tire.LFZO *...
            obj.d_tire.LKY;

            mufy = (obj.d_tire.PDY1 + obj.d_tire.PDY2 * Dffz) * obj.d_tire.LMUY;
            Dfy = mufy * Ffz;

            Cfy = obj.d_tire.PCY1 * obj.d_tire.LCY;

            Bfy = Kfy / (Cfy * Dfy);

            Efy = (obj.d_tire.PEY1 + obj.d_tire.PEY2 * Dffz) * obj.d_tire.LEY;
            
            % rear wheel coefficients
            Kry = obj.d_tire.PKY1 * fzNominal *...
            sin(2.0 * atan2(Frz, (obj.d_tire.PKY2 * fzNominal * obj.d_tire.LFZO))) * obj.d_tire.LFZO *...
            obj.d_tire.LKY;

            mury = (obj.d_tire.PDY1 + obj.d_tire.PDY2 * Drfz) * obj.d_tire.LMUY;
            Dry = mury * Frz;

            Cry = obj.d_tire.PCY1 * obj.d_tire.LCY;

            Bry = Kry / (Cry * Dry);

            Ery = (obj.d_tire.PEY1 + obj.d_tire.PEY2 * Drfz) * obj.d_tire.LEY;

            % latteral front force
            Ffy = 2*Dfy * sin(Cfy * atan(Bfy * saf - Efy * (Bfy * saf - atan(Bfy * saf))));

            % latteral rear force
            Fry = 2*Dry * sin(Cry * atan(Bry * sar - Ery * (Bry * sar - atan(Bry * sar))));

            rhs = [vx * cos(yaw) - vy * sin(yaw);
                   vx * sin(yaw) + vy * cos(yaw);
                   r;
                   1 / obj.d_car.m *...
                   (Frx + cos(steeringAngle) * Ffx + Fdrag - sin(steeringAngle) * Ffy +...
                    obj.d_car.m * vy * r);
                   1 / obj.d_car.m *...
                   (Fry + cos(steeringAngle) * Ffy + sin(steeringAngle) * Ffx - obj.d_car.m * vx * r);
                   1 / obj.d_car.iz *...
                   (-Fry * obj.d_car.lr + (cos(steeringAngle) * Ffy + sin(steeringAngle) * Ffx) * obj.d_car.lf);
                   vs;
                   dThrottle;
                   dSteeringAngle;
                   dBrakes;
                   dVs];

            obj.f = Function('f',{states,controls},{rhs});
            obj.U = SX.sym('U',obj.NU,obj.N);
            % obj.P = SX.sym('P',obj.NX + 2 * obj.N + obj.NU * obj.N);
            obj.X = SX.sym('X',obj.NX,(obj.N+1));
            % slack variables matrix for soft constraints
            obj.S = SX.sym('S',obj.NS,(obj.N+1));
            
            % objective function
            obj.objective = 0;

            % constraints
            obj.g = [];
            
            % Coeffs for laf and contouring errors penallization
            obj.Q = zeros(2,2);
            obj.Q(1,1) = obj.d_parameters.costs.qC;
            obj.Q(2,2) = obj.d_parameters.costs.qL;
            
            % Coeffs for control inputs penalization
            obj.R = zeros(obj.NU,obj.NU);
            obj.R(1,1) = obj.d_parameters.costs.rThrottle;
            obj.R(2,2) = obj.d_parameters.costs.rSteeringAngle;
            obj.R(3,3) = obj.d_parameters.costs.rBrakes;
            obj.R(4,4) = obj.d_parameters.costs.rVs;

            % Coeffs for soft constraints penalization
            obj.Z = zeros(obj.NS,obj.NS);
            obj.Z(1,1) = obj.d_parameters.costs.scQuadAlpha;
            obj.Z(2,2) = obj.d_parameters.costs.scQuadAlpha;
        end

        function obj = computeConstraints(obj)
            x0 = obj.X(:,1);
            obj.g = [obj.g;x0-obj.P(1:obj.NX)];
            for i = 1:obj.N
                state = obj.X(:,i);
                stateNext = obj.X(:,i+1);
                control = obj.U(:,i);

                % rk4 integration
                k1 = obj.f(state,control);
                k2 = obj.f(state + obj.d_ts/2 * k1,control);
                k3 = obj.f(state + obj.d_ts/2 * k2,control);
                k4 = obj.f(state + obj.d_ts * k3,control);
                stateNextRK4 = state + obj.d_ts/6 * (k1 + 2*k2 + 2*k3 + k4);
                
                % base constraint
                obj.g = [obj.g, stateNext - stateNextRK4];
            end

            %Track constraint and slip angles constraints

            for i = 1:obj.N+1

                state = obj.X(:,i);

                x = state(1);
                y = state(2);

                vx = state(4);
                vy = state(5);
                r = state(6);
                steeringAngle = state(9);

                % track constraint: lowerBound <= ax-by <= upperBound,
                % where a = P(NX + 2*i -1) and b = P(NX + 2*i)
                obj.g = [obj.g, obj.P(obj.NX + 2*i-1)*x - obj.P(obj.NX + 2*i)*y];

                % front slip angle constraint with slack variable
                safg = atan2((vy + r*lf),vx) - steeringAngle + obj.S(1,i);
                obj.g = [obj.g,safg];  

                % rear slip angle constraint with slack variable
                sarg = atan2((vy - r*lr),vx) + obj.S(2,i);
                obj.g = [obj.g,sarg];     
            end 
        end

        function point = getRefPoint(obj,s)
            % compute all the geometry information of the track at a given arc length
            % X-Y postion of the reference at s
            posRef = obj.d_track.getPosition(s);
            xRef = posRef(1);
            yRef = posRef(2);
            % reference path derivatives
            dposRef = obj.d_track.getDerivative(s);
            dxRef = dposRef(1);
            dyRef = dposRef(2);
            % angle of the reference path
            thetaRef = atan2(dyRef, dxRef);
            % second order derivatives
            ddposRef = obj.d_track.getSecondDerivative(s);
            ddxRef = ddposRef(1);
            ddyRef = ddposRef(2);
            % curvature
            dthetaRefNom = (dxRef * ddyRef - dyRef * ddxRef);
            dthetaRefDenom = (dxRef * dxRef + dyRef * dyRef);
            dthetaRef = dthetaRefNom / dthetaRefDenom;
            point = TrackPoint(xRef, yRef, dxRef, dyRef, thetaRef, dthetaRef);
        end

        function obj = computeCost(obj)
            for i = 1:obj.N
                state = obj.X(:,i);
                control = obj.U(:,i);
                
                x = state(1);
                y = state(2);
                s = state(7);
                vs = state(11);


                trackPoint = getRefPoint(s);
                
                % contouring error
                ec = -sin(trackPoint.thetaRef) * (trackPoint.xRef - x)...
                                    + cos(trackPoint.thetaRef) * (trackPoint.yRef - y);
                % lag error
                el = cos(trackPoint.thetaRef) * (trackPoint.xRef - x)...
                                    + sin(trackPoint.thetaRef) * (trackPoint.yRef - y);
                error = [ec;el];

                % soft constraints slack variables matrix
                slacks = obj.S(:,i);
                
                % objective function
                obj.objective = obj.objective + error' * obj.Q * error + ...
                    control' * obj.R * control - obj.d_parameters.qVs * vs + slacks' *obj.Z * slacks;
            end 
        end

        function obj = initIpoptSolver(obj)
            % make the decision variable one column  vector
            OPT_variables = [reshape(obj.X,obj.NX*(obj.N+1),1);reshape(obj.U,obj.NU*obj.N,1);reshape(obj.S,obj.NS*(obj.N+1),1)];
            nlpProb = struct('f', obj.objective, 'x', OPT_variables, 'g', obj.g, 'p', obj.P);

            obj.opts = struct;
            obj.opts.ipopt.max_iter = 2000;
            obj.opts.ipopt.print_level =0;%0,3
            % obj.opts.verbose = 
            obj.opts.jit = true;
            obj.opts.print_time = 0;
            obj.opts.ipopt.acceptable_tol =1e-8;
            obj.opts.ipopt.acceptable_obj_change_tol = 1e-6;
            obj.opts.ipopt.fixed_variable_treatment = "make_parameter";
            % obj.opts.ipopt.linear_solver = "ma57";

            obj.solver = nlpsol('solver','ipopt',nlpProb,obj.opts);
        end

        function obj = initConstraints(obj)
            obj.lbx = zeros(obj.NX * (obj.N+1) + obj.NU * (obj.N) + obj.NS * (obj.N+1) ,1);
            obj.ubx = zeros(obj.NX * (obj.N+1) + obj.NU * (obj.N) + obj.NS * (obj.N+1) ,1);
            
            for i = 1:obj.N+1
                obj.lbx((obj.NX*(i-1) + 1):obj.NX*i,1) = [obj.d_parameters.bounds.lowerStateBounds.xL,...
                                                          obj.d_parameters.bounds.lowerStateBounds.yL,...
                                                          obj.d_parameters.bounds.lowerStateBounds.phiL,...
                                                          obj.d_parameters.bounds.lowerStateBounds.vxL,...
                                                          obj.d_parameters.bounds.lowerStateBounds.vyL,...
                                                          obj.d_parameters.bounds.lowerStateBounds.rL,...
                                                          obj.d_parameters.bounds.lowerStateBounds.sL,...
                                                          obj.d_parameters.bounds.lowerStateBounds.throttleL,...
                                                          obj.d_parameters.bounds.lowerStateBounds.steeringAngleL,...
                                                          obj.d_parameters.bounds.lowerStateBounds.brakesL,...
                                                          obj.d_parameters.bounds.lowerStateBounds.vsL];

                obj.ubx((obj.NX*(i-1) + 1):obj.NX*i,1) = [obj.d_parameters.bounds.upperStateBounds.xU,...
                                                          obj.d_parameters.bounds.upperStateBounds.yU,...
                                                          obj.d_parameters.bounds.upperStateBounds.phiU,...
                                                          obj.d_parameters.bounds.upperStateBounds.vxU,...
                                                          obj.d_parameters.bounds.upperStateBounds.vyU,...
                                                          obj.d_parameters.bounds.upperStateBounds.rU,...
                                                          obj.d_parameters.bounds.upperStateBounds.sU,...
                                                          obj.d_parameters.bounds.upperStateBounds.throttleU,...
                                                          obj.d_parameters.bounds.upperStateBounds.steeringAngleU,...
                                                          obj.d_parameters.bounds.upperStateBounds.brakesU,...
                                                          obj.d_parameters.bounds.upperStateBounds.vsU];
            end
            for i = 1:obj.N
               obj.lbx((obj.NX*(obj.N+1) + obj.NU*(i-1) + 1):obj.NX*(obj.N+1) + obj.NU*i,1) = ...
                   [obj.d_parameters.bounds.lowerInputBounds.dThrottleL,...
                    obj.d_parameters.bounds.lowerInputBounds.dSteeringAngleL,...
                    obj.d_parameters.bounds.lowerInputBounds.dBrakesL,...
                    obj.d_parameters.bounds.lowerInputBounds.dVsL];

               obj.ubx((obj.NX*(obj.N+1) + obj.NU*(i-1) + 1):obj.NX*(obj.N+1) + obj.NU*i,1) = ...
                   [obj.d_parameters.bounds.upperInputBounds.dThrottleU,...
                    obj.d_parameters.bounds.upperInputBounds.dSteeringAngleU,...
                    obj.d_parameters.bounds.upperInputBounds.dBrakesU,...
                    obj.d_parameters.bounds.upperInputBounds.dVsU]; 
            end
            for i = 1:obj.N+1
                obj.lbx((obj.NX*(obj.N+1) + obj.NU*obj.N + obj.NS*(i-1) + 1):(obj.NX*(obj.N+1) + obj.NU*obj.N + obj.NS*i),1) = ...
                    [obj.d_parameters.bounds.]
                obj.ubx((obj.NX*(obj.N+1) + obj.NU*obj.N + obj.NS*(i-1) + 1):(obj.NX*(obj.N+1) + obj.NU*obj.N + obj.NS*i),1) = ...
            end
        end

        function ipoptReturn = solve(obj, x0)
            
        end
    end
end


classdef IpoptCasadi < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        d_config
        d_parameters
        d_ts
        d_car
        d_tire

        d_track
        d_outerBorder
        d_innerBorder

        d_trackInterpolation;
        d_outerBorderInterpolation;
        d_innerBorderInterpolation;

        X
        U
        P
        S
        f
        g
        p
        objective

        Q
        R
        Z

        solver
        opts

        d_initialStateGuess
        d_initialControlGuess

        d_validInitialGuess

        args
    end
    
    methods (Access = public)
        function obj = IpoptCasadi(config,parameters)
            obj.d_config = config;
            obj.d_parameters = parameters;
            obj.d_ts = parameters.config.ts;
            obj.d_car = parameters.car;
            obj.d_tire = parameters.tire;
            obj.d_validInitialGuess = false;

            obj.d_track.centerLine = ArcLengthSpline(config,parameters.mpcModel);
            obj.d_track.outerBorder = ArcLengthSpline(config,parameters.mpcModel);
            obj.d_track.innerBorder = ArcLengthSpline(config,parameters.mpcModel);
        end

        function initMPC(obj)
            obj.initSystemModel();
            obj.initEqualityConstraints();
            %obj.initInequalityConstraints();
            obj.initCostFunction();
            obj.initIpoptSolver();
            obj.setBounds();
        end

        function setTrack(obj,track)
            import casadi.*;

            obj.d_track.centerLine.gen2DSpline(track.x,track.y);
            obj.d_track.outerBorder.gen2DSpline(track.xOuter,track.yOuter);
            obj.d_track.innerBorder.gen2DSpline(track.xInner,track.yInner);

            centerLine = obj.d_track.centerLine.getPath();
            outerBoredr = obj.d_track.outerBorder.getPath();
            innerBorder = obj.d_track.innerBorder.getPath();

            centerLineDerivatives = zeros(2,length(centerLine.s));
            
            for i = 1:length(centerLine.s)
                centerLineDerivatives(:,i) = obj.d_track.centerLine.getDerivative(centerLine.s(i));
            end
            
            obj.d_track.centerLineInterpolation.x = interpolant('center_line_interpolation_x','bspline',{centerLine.s},centerLine.x);
            obj.d_track.centerLineInterpolation.y = interpolant('center_line_interpolation_y','bspline',{centerLine.s},centerLine.y);
            obj.d_track.centerLineDerivativesInterpolation.x = interpolant('center_line_derivative_interpolation_x','bspline',{centerLine.s},centerLineDerivatives(1,:));
            obj.d_track.centerLineDerivativesInterpolation.y = interpolant('center_line_derivative_interpolation_y','bspline',{centerLine.s},centerLineDerivatives(2,:));

            obj.d_track.outerBorderInterpolation.x = interpolant('outerBorder_interpolation_x','bspline',{outerBoredr.s},outerBoredr.x);
            obj.d_track.outerBorderInterpolation.y = interpolant('outerBorder_interpolation_y','bspline',{outerBoredr.s},outerBoredr.y);

            obj.d_track.innerBorderInterpolation.x = interpolant('innerBorder_interpolation_x','bspline',{innerBorder.s},innerBorder.x);
            obj.d_track.innerBorderInterpolation.y = interpolant('innerBorder_interpolation_y','bspline',{innerBorder.s},innerBorder.y);
        end

        function track = getTrack(obj)
            track = obj.d_track.centerLine;
        end

        function rhs = initKinematicModel(obj, states, controls)
            yaw = states(3);
            vx = states(4);
            vy = states(5);
            r = states(6);
            throttle = states(8);
            steeringAngle = states(9);
            brakes = states(10);
            vs = states(11);

            dThrottle = controls(1);
            dSteeringAngle = controls(2);
            dBrakes = controls(3);
            dVs = controls(4);
            
            % car parameters
            rDyn = obj.d_car.rDyn;
            
            cdrv = obj.d_car.cm1*obj.d_car.gearRatio;
            cbf = obj.d_car.cbf;
            cbr = obj.d_car.cbr;

            m = obj.d_car.m;
            lf = obj.d_car.lf;
            lr = obj.d_car.lr;
            gAcc = obj.d_car.g;

            % normal load on the one front wheel
            Ffz = lr*m*gAcc/(2.0*(lf+lr));

            % normal load on the one rear wheel
            Frz = lf*m*gAcc/(2.0*(lf+lr));
            
            % rolling resistance of the two front wheels
            Ffrr = 2*obj.d_tire.QSY1*Ffz;

            % rolling resistance of the two rear wheels
            Frrr = 2*obj.d_tire.QSY1*Frz;

            % longitudinal front force
            Ffx = (-cbf*brakes)/rDyn + Ffrr;
            
            % longitudinal rear force
            Frx = (-cbr*brakes+cdrv*throttle)/rDyn + Frrr;

            % drag force
            Fdrag = obj.d_car.cd*vx^2.0;

            % dot vx
            vxDot = 1/m*(Frx + cos(steeringAngle)*Ffx+Fdrag);

            rhs = [vx*cos(yaw)-vy*sin(yaw);
                   vx*sin(yaw)+vy*cos(yaw);
                   r;
                   vxDot;
                   lr/(lr+lf)*(dSteeringAngle*vx+steeringAngle*vxDot);
                   1/(lr+lf)*(dSteeringAngle*vx+steeringAngle*vxDot);
                   vs;
                   dThrottle;
                   dSteeringAngle;
                   dBrakes;
                   dVs];
        end

        function rhs = initDynamicModel(obj, states, controls)
            yaw = states(3);
            vx = states(4);
            vy = states(5);
            r = states(6);
            throttle = states(8);
            steeringAngle = states(9);
            brakes = states(10);
            vs = states(11);

            dThrottle = controls(1);
            dSteeringAngle = controls(2);
            dBrakes = controls(3);
            dVs = controls(4);

            %Dynamic forces
            rDyn = obj.d_car.rDyn;
            
            cdrv = obj.d_car.cm1 * obj.d_car.gearRatio;
            cbf = obj.d_car.cbf;
            cbr = obj.d_car.cbr;

            m = obj.d_car.m;
            iz = obj.d_car.iz;
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
            
            % drag force
            Fdrag = obj.d_car.cd * vx^2.0;

            rhs = [vx * cos(yaw) - vy * sin(yaw);
                   vx * sin(yaw) + vy * cos(yaw);
                   r;
                   1 / m *...
                   (Frx + cos(steeringAngle) * Ffx + Fdrag - sin(steeringAngle) * Ffy +...
                   m * vy * r);
                   1 / m *...
                   (Fry + cos(steeringAngle) * Ffy + sin(steeringAngle) * Ffx - m * vx * r);
                   1 / iz *...
                   (-Fry * lr + (cos(steeringAngle) * Ffy + sin(steeringAngle) * Ffx) * lf);
                   vs;
                   dThrottle;
                   dSteeringAngle;
                   dBrakes;
                   dVs];
        end

        function initSystemModel(obj)
            % States
            import casadi.*;

            x = MX.sym('x');
            y = MX.sym('y');
            yaw = MX.sym('yaw');
            vx = MX.sym('vx');
            vy = MX.sym('vy');
            r = MX.sym('r');
            s = MX.sym('s');
            throttle = MX.sym('throttle');
            steeringAngle = MX.sym('steeringAngle');
            brakes = MX.sym('brakes');
            vs = MX.sym('vs');
            
            states = [x;y;yaw;vx;vy;r;s;throttle;steeringAngle;brakes;vs];
            
            % Controls
            dThrottle = MX.sym('dThrottle');
            dSteeringAngle = MX.sym('dSteeringAngle');
            dBrakes = MX.sym('dBrakes');
            dVs = MX.sym('dVs');

            controls = [dThrottle,dSteeringAngle,dBrakes,dVs];

            rhs = obj.initKinematicModel(states,controls);
            % rhs = obj.initDynamicModel(states,controls);

            obj.f = Function('f',{states,controls},{rhs});
            
            obj.U = MX.sym('U',obj.d_config.NU,obj.d_config.N);
            obj.P = MX.sym('P',obj.d_config.NX,1); % NX - initial state
            obj.X = MX.sym('X',obj.d_config.NX,(obj.d_config.N+1));
            % slack variables matrix for soft constraints
            %obj.S = MX.sym('S',obj.d_config.NS,(obj.d_config.N+1));
            
            % objective function
            obj.objective = 0;

            % constraints
            obj.g = [];
            
            % Coeffs for laf and contouring errors penallization
            obj.Q = zeros(2,2);
            obj.Q(1,1) = obj.d_parameters.costs.qC;
            obj.Q(2,2) = obj.d_parameters.costs.qL;
            
            % Coeffs for control inputs penalization
            obj.R = zeros(obj.d_config.NU,obj.d_config.NU);
            obj.R(1,1) = obj.d_parameters.costs.rThrottle;
            obj.R(2,2) = obj.d_parameters.costs.rSteeringAngle;
            obj.R(3,3) = obj.d_parameters.costs.rBrakes;
            obj.R(4,4) = obj.d_parameters.costs.rVs;

            % Coeffs for soft constraints penalization
            %obj.Z = zeros(obj.d_config.NS,obj.d_config.NS);
            %obj.Z(1,1) = obj.d_parameters.costs.scQuadAlpha;
            %obj.Z(2,2) = obj.d_parameters.costs.scQuadAlpha;
            %obj.Z(3,3) = obj.d_parameters.costs.scQuadTrack;
        end

        function initEqualityConstraints(obj)
            x0 = obj.X(:,1);
            obj.g = [obj.g;x0-obj.P(1:obj.d_config.NX)];
            for i = 1:obj.d_config.N
                state = obj.X(:,i);
                stateNext = obj.X(:,i+1);
                control = obj.U(:,i);

                % rk4 integration
                stateNextRK4 = obj.ode4(state,control);
                
                % base constraint
                obj.g = [obj.g;stateNext-stateNextRK4];
            end
        end

        function initInequalityConstraints(obj)
            for i = 1:obj.d_config.N+1

                state = obj.X(:,i);
    
                x = state(1);
                y = state(2);
                vx = state(4);
                vy = state(5);
                r = state(6);
                s = state(7);
                steeringAngle = state(9);
    
                lf = obj.d_car.lf;
                lr = obj.d_car.lr;
    
                % front slip angle constraint with slack variable
                safg = atan2((vy + r*lf),vx) - steeringAngle + obj.S(1,i);
                obj.g = [obj.g;safg];  
    
                % rear slip angle constraint with slack variable
                sarg = atan2((vy - r*lr),vx) + obj.S(2,i);
                obj.g = [obj.g;sarg];  
    
                % track constraint: 0.0 <= (X - Xcen(S))^2 + (Y - Ycen(S))^2 <= rOut
                obj.g = [obj.g;(x-obj.d_track.centerLineInterpolation.x(s))^2 + (y-obj.d_track.centerLineInterpolation.y(s))^2 + obj.S(3,i)];

                % soft constraints slack variables matrix
                slacks = obj.S(:,i);
                
                % objective function with slack vars
                obj.objective = obj.objective + slacks' *obj.Z * slacks;
            end 
        end

        function initCostFunction(obj)
            for i = 1:obj.d_config.N
                stateNext = obj.X(:,i+1);
                control = obj.U(:,i);
                
                x = stateNext(1);
                y = stateNext(2);
                s = stateNext(7);
                vs = stateNext(11);
                
                xRef = obj.d_track.centerLineInterpolation.x(s);
                yRef = obj.d_track.centerLineInterpolation.y(s);
                thetaRef = atan2(obj.d_track.centerLineDerivativesInterpolation.y(s),obj.d_track.centerLineDerivativesInterpolation.x(s));
                
                % contouring error
                ec = -sin(thetaRef) * (xRef - x)...
                                    + cos(thetaRef) * (yRef - y);
                % lag error
                el = cos(thetaRef) * (xRef - x)...
                                    + sin(thetaRef) * (yRef - y);
                error = [ec;el];
                
                % objective function
                obj.objective = obj.objective + error' * obj.Q * error + ...
                    control' * obj.R * control - obj.d_parameters.costs.qVs * vs;
            end 
        end

        function initIpoptSolver(obj)
            import casadi.*;

            % make the decision variable one column  vector
            OPT_variables = [reshape(obj.X,obj.d_config.NX*(obj.d_config.N+1),1);reshape(obj.U,obj.d_config.NU*obj.d_config.N,1)];%reshape(obj.S,obj.d_config.NS*(obj.d_config.N+1),1)];
            nlpProb = struct('f', obj.objective, 'x', OPT_variables, 'g', obj.g, 'p', obj.P);

            obj.opts = struct;
            obj.opts.ipopt.max_iter = 2000;
            obj.opts.ipopt.print_level = 3;%0,3
            obj.opts.print_time = 0;
            obj.opts.compiler = 'shell';
            obj.opts.jit = true;
            obj.opts.ipopt.acceptable_tol =1e-8;
            obj.opts.ipopt.acceptable_obj_change_tol = 1e-6;
            
            obj.solver = nlpsol('solver','ipopt',nlpProb,obj.opts);
        end

        function setBounds(obj)
            obj.args.lbx = zeros(obj.d_config.NX * (obj.d_config.N+1) + obj.d_config.NU * (obj.d_config.N) + obj.d_config.NS*(obj.d_config.N+1),1); % lower bounds for states, inputs and slack vars
            obj.args.ubx = zeros(obj.d_config.NX * (obj.d_config.N+1) + obj.d_config.NU * (obj.d_config.N) + obj.d_config.NS*(obj.d_config.N+1),1); % upper bounds for states, inputs and slack vars

            obj.args.lbg = zeros(obj.d_config.NX*(obj.d_config.N+1) + obj.d_config.NS*(obj.d_config.N+1),1); % lower bounds for equality and inequality constraints
            obj.args.ubg = zeros(obj.d_config.NX*(obj.d_config.N+1) + obj.d_config.NS*(obj.d_config.N+1),1); % upper bounds for equality and inequality constraints

            % lower and upper bounds vals for states, inputs and slack variables
            % lower and upper bounds for states
          
            obj.args.lbx(1:obj.d_config.NX:obj.d_config.NX*(obj.d_config.N+1),1) = obj.d_parameters.bounds.lowerStateBounds.xL;
            obj.args.lbx(2:obj.d_config.NX:obj.d_config.NX*(obj.d_config.N+1),1) = obj.d_parameters.bounds.lowerStateBounds.yL;
            obj.args.lbx(3:obj.d_config.NX:obj.d_config.NX*(obj.d_config.N+1),1) = obj.d_parameters.bounds.lowerStateBounds.phiL;
            obj.args.lbx(4:obj.d_config.NX:obj.d_config.NX*(obj.d_config.N+1),1) = obj.d_parameters.bounds.lowerStateBounds.vxL;
            obj.args.lbx(5:obj.d_config.NX:obj.d_config.NX*(obj.d_config.N+1),1) = obj.d_parameters.bounds.lowerStateBounds.vyL;
            obj.args.lbx(6:obj.d_config.NX:obj.d_config.NX*(obj.d_config.N+1),1) = obj.d_parameters.bounds.lowerStateBounds.rL;
            obj.args.lbx(7:obj.d_config.NX:obj.d_config.NX*(obj.d_config.N+1),1) = obj.d_parameters.bounds.lowerStateBounds.sL;
            obj.args.lbx(8:obj.d_config.NX:obj.d_config.NX*(obj.d_config.N+1),1) = obj.d_parameters.bounds.lowerStateBounds.throttleL;
            obj.args.lbx(9:obj.d_config.NX:obj.d_config.NX*(obj.d_config.N+1),1) = obj.d_parameters.bounds.lowerStateBounds.steeringAngleL;
            obj.args.lbx(10:obj.d_config.NX:obj.d_config.NX*(obj.d_config.N+1),1) = obj.d_parameters.bounds.lowerStateBounds.brakesL;
            obj.args.lbx(11:obj.d_config.NX:obj.d_config.NX*(obj.d_config.N+1),1) = obj.d_parameters.bounds.lowerStateBounds.vsL;

            obj.args.ubx(1:obj.d_config.NX:obj.d_config.NX*(obj.d_config.N+1),1) = obj.d_parameters.bounds.upperStateBounds.xU;
            obj.args.ubx(2:obj.d_config.NX:obj.d_config.NX*(obj.d_config.N+1),1) = obj.d_parameters.bounds.upperStateBounds.yU;
            obj.args.ubx(3:obj.d_config.NX:obj.d_config.NX*(obj.d_config.N+1),1) = obj.d_parameters.bounds.upperStateBounds.phiU;
            obj.args.ubx(4:obj.d_config.NX:obj.d_config.NX*(obj.d_config.N+1),1) = obj.d_parameters.bounds.upperStateBounds.vxU;
            obj.args.ubx(5:obj.d_config.NX:obj.d_config.NX*(obj.d_config.N+1),1) = obj.d_parameters.bounds.upperStateBounds.vyU;
            obj.args.ubx(6:obj.d_config.NX:obj.d_config.NX*(obj.d_config.N+1),1) = obj.d_parameters.bounds.upperStateBounds.rU;
            obj.args.ubx(7:obj.d_config.NX:obj.d_config.NX*(obj.d_config.N+1),1) = obj.d_parameters.bounds.upperStateBounds.sU;
            obj.args.ubx(8:obj.d_config.NX:obj.d_config.NX*(obj.d_config.N+1),1) = obj.d_parameters.bounds.upperStateBounds.throttleU;
            obj.args.ubx(9:obj.d_config.NX:obj.d_config.NX*(obj.d_config.N+1),1) = obj.d_parameters.bounds.upperStateBounds.steeringAngleU;
            obj.args.ubx(10:obj.d_config.NX:obj.d_config.NX*(obj.d_config.N+1),1) = obj.d_parameters.bounds.upperStateBounds.brakesU;
            obj.args.ubx(11:obj.d_config.NX:obj.d_config.NX*(obj.d_config.N+1),1) = obj.d_parameters.bounds.upperStateBounds.vsU;
           

            % lower and upper bounds for inputs
            
            obj.args.lbx(obj.d_config.NX*(obj.d_config.N+1)+1: ...
                                          obj.d_config.NU: ...
                                          obj.d_config.NX*(obj.d_config.N+1)+obj.d_config.NU*obj.d_config.N,1) = obj.d_parameters.bounds.lowerInputBounds.dThrottleL;

            obj.args.lbx(obj.d_config.NX*(obj.d_config.N+1)+2: ...
                                          obj.d_config.NU: ...
                                          obj.d_config.NX*(obj.d_config.N+1)+obj.d_config.NU*obj.d_config.N,1) = obj.d_parameters.bounds.lowerInputBounds.dSteeringAngleL;

            obj.args.lbx(obj.d_config.NX*(obj.d_config.N+1)+3: ...
                                          obj.d_config.NU: ...
                                          obj.d_config.NX*(obj.d_config.N+1)+obj.d_config.NU*obj.d_config.N,1) = obj.d_parameters.bounds.lowerInputBounds.dBrakesL;

            obj.args.lbx(obj.d_config.NX*(obj.d_config.N+1)+4: ...
                                          obj.d_config.NU: ...
                                          obj.d_config.NX*(obj.d_config.N+1)+obj.d_config.NU*obj.d_config.N,1) = obj.d_parameters.bounds.lowerInputBounds.dVsL;


            obj.args.ubx(obj.d_config.NX*(obj.d_config.N+1)+1: ...
                                          obj.d_config.NU: ...
                                          obj.d_config.NX*(obj.d_config.N+1)+obj.d_config.NU*obj.d_config.N,1) = obj.d_parameters.bounds.upperInputBounds.dThrottleU;

            obj.args.ubx(obj.d_config.NX*(obj.d_config.N+1)+2: ...
                                          obj.d_config.NU: ...
                                          obj.d_config.NX*(obj.d_config.N+1)+obj.d_config.NU*obj.d_config.N,1) = obj.d_parameters.bounds.upperInputBounds.dSteeringAngleU;

            obj.args.ubx(obj.d_config.NX*(obj.d_config.N+1)+3: ...
                                          obj.d_config.NU: ...
                                          obj.d_config.NX*(obj.d_config.N+1)+obj.d_config.NU*obj.d_config.N,1) = obj.d_parameters.bounds.upperInputBounds.dBrakesU;

            obj.args.ubx(obj.d_config.NX*(obj.d_config.N+1)+4: ...
                                          obj.d_config.NU: ...
                                          obj.d_config.NX*(obj.d_config.N+1)+obj.d_config.NU*obj.d_config.N,1) = obj.d_parameters.bounds.upperInputBounds.dVsU;
           
            % lower and upper bounds for equality and inequality constraints
            % lower and upper bounds for the equality constraints
            obj.args.lbg(1:obj.d_config.NX*(obj.d_config.N+1)) = 0;
            obj.args.ubg(1:obj.d_config.NX*(obj.d_config.N+1)) = 0;
            % lower and upper bounds for the inequality constraints
            % lower and upper bounds for the front slip angle
            obj.args.lbg(obj.d_config.NX*(obj.d_config.N+1) + 1:obj.d_config.NS:obj.d_config.NX*(obj.d_config.N+1) + obj.d_config.NS*(obj.d_config.N+1)) = -obj.d_parameters.mpcModel.maxAlpha;
            obj.args.ubg(obj.d_config.NX*(obj.d_config.N+1) + 1:obj.d_config.NS:obj.d_config.NX*(obj.d_config.N+1) + obj.d_config.NS*(obj.d_config.N+1)) = obj.d_parameters.mpcModel.maxAlpha;
            % lower and upper bounds for the rear slip angle
            obj.args.lbg(obj.d_config.NX*(obj.d_config.N+1) + 2:obj.d_config.NS:obj.d_config.NX*(obj.d_config.N+1) + obj.d_config.NS*(obj.d_config.N+1)) = -obj.d_parameters.mpcModel.maxAlpha;
            obj.args.ubg(obj.d_config.NX*(obj.d_config.N+1) + 2:obj.d_config.NS:obj.d_config.NX*(obj.d_config.N+1) + obj.d_config.NS*(obj.d_config.N+1)) = obj.d_parameters.mpcModel.maxAlpha;
            % lower and upper bounds for the track constraint
            obj.args.lbg(obj.d_config.NX*(obj.d_config.N+1) + 3:obj.d_config.NS:obj.d_config.NX*(obj.d_config.N+1) + obj.d_config.NS*(obj.d_config.N+1)) = 0.0;
            obj.args.ubg(obj.d_config.NX*(obj.d_config.N+1) + 3:obj.d_config.NS:obj.d_config.NX*(obj.d_config.N+1) + obj.d_config.NS*(obj.d_config.N+1)) = obj.d_parameters.mpcModel.rOut;
        end

        function ipoptReturn = runMPC(obj, x0)
            %s0 = zeros(obj.d_config.NS,obj.d_config.N+1);
            x0 = obj.unwrapState(x0);
            x0(obj.d_config.siIndex.s) = obj.d_track.centerLine.projectOnSpline(vectorToState(x0));
            
            if obj.d_validInitialGuess
              obj.updateInitialGuess2(x0);
            else
              obj.generateNewInitialGuess2(x0);
            end

            obj.args.p(1:obj.d_config.NX,1) = x0;
          
            obj.args.x0 = [reshape(obj.d_initialStateGuess,obj.d_config.NX*(obj.d_config.N+1),1);
                           reshape(obj.d_initialControlGuess,obj.d_config.NU*(obj.d_config.N),1)];
                           %reshape(s0,obj.d_config.NS*(obj.d_config.N+1),1)];
            
            sol = obj.solver('x0', obj.args.x0, 'lbx', obj.args.lbx, 'ubx', obj.args.ubx,...
            'lbg', obj.args.lbg, 'ubg', obj.args.ubg,'p',obj.args.p);
            
            obj.d_initialStateGuess = reshape(full(sol.x(1:obj.d_config.NX*(obj.d_config.N+1)))', ...
                                   obj.d_config.NX,obj.d_config.N+1);
            obj.d_initialControlGuess = reshape(full(sol.x(obj.d_config.NX*(obj.d_config.N+1)+1:obj.d_config.NX*(obj.d_config.N+1)+obj.d_config.NU*obj.d_config.N))', ...
                                   obj.d_config.NU,obj.d_config.N);

            ipoptReturn = IpoptReturn;

            ipoptReturn.x0 = obj.d_initialStateGuess(:,1);
            ipoptReturn.u0 = obj.d_initialControlGuess(:,1);
            ipoptReturn.mpcHorizon = obj.d_initialStateGuess;
        end

        function x0 = unwrapState(obj,x0)
            trackLength = obj.d_track.centerLine.getLength();
            if x0(obj.d_config.siIndex.yaw) > pi
              x0(obj.d_config.siIndex.yaw) = x0(obj.d_config.siIndex.yaw) - 2.0 * pi;
            end
            if x0(obj.d_config.siIndex.yaw) < -pi
              x0(obj.d_config.siIndex.yaw) = x0(obj.d_config.siIndex.yaw) + 2.0 * pi;
            end
            if x0(obj.d_config.siIndex.s) > trackLength
              x0(obj.d_config.siIndex.s) = x0(obj.d_config.siIndex.s) - trackLength;
            end
            if x0(obj.d_config.siIndex.s) < 0.0
              x0(obj.d_config.siIndex.s) = x0(obj.d_config.siIndex.s) + trackLength;
            end
        end

        function updateInitialGuess2(obj,x0)
            for i = 2:obj.d_config.N 
                obj.d_initialStateGuess(:,i-1) = obj.d_initialStateGuess(:,i);
                obj.d_initialControlGuess(:,i-1) = obj.d_initialControlGuess(:,i);
            end
            obj.d_initialStateGuess(:,1) = x0;
            obj.d_initialStateGuess(:,obj.d_config.N) = obj.d_initialStateGuess(:,obj.d_config.N+1);
            obj.d_initialControlGuess(:,obj.d_config.N) = zeros(obj.d_config.NU,1);
            obj.d_initialStateGuess(:,obj.d_config.N+1) = obj.d_initialStateGuess(:,obj.d_config.N);
            obj.unwrapInitialGuess();
        end

        function updateInitialGuess(obj,x0)
            for i = 2:obj.d_config.N 
                obj.d_initialStateGuess(:,i-1) = obj.d_initialStateGuess(:,i);
                obj.d_initialControlGuess(:,i-1) = obj.d_initialControlGuess(:,i);
            end

            obj.d_initialStateGuess(:,1) = x0;
            
            obj.d_initialStateGuess(:,obj.d_config.N) = obj.d_initialStateGuess(:,obj.d_config.N-1); 
            obj.d_initialControlGuess(:,obj.d_config.N) = obj.d_initialControlGuess(:,obj.d_config.N-1);
            obj.d_initialStateGuess(:,obj.d_config.N+1) = obj.ode4(obj.d_initialStateGuess(:,obj.d_config.N),obj.d_initialControlGuess(:,obj.d_config.N)).full();
            obj.unwrapInitialGuess();
        end
        
        function generateNewInitialGuess2(obj,x0)
            obj.d_initialStateGuess = zeros(obj.d_config.NX,obj.d_config.N+1);
            obj.d_initialStateGuess(:,1) = x0;
            obj.d_initialControlGuess = zeros(obj.d_config.NU,obj.d_config.N);
            obj.unwrapInitialGuess();
            obj.d_validInitialGuess = true;
        end

        function generateNewInitialGuess(obj,x0)
            obj.d_initialStateGuess = zeros(obj.d_config.NX,obj.d_config.N+1);
            obj.d_initialControlGuess = zeros(obj.d_config.NU,obj.d_config.N);

            obj.d_initialStateGuess(:,1) = x0;
            obj.d_initialStateGuess(obj.d_config.siIndex.vx,1) = obj.d_parameters.mpcModel.initialVelocity;
            obj.d_initialStateGuess(obj.d_config.siIndex.vs,1) = obj.d_parameters.mpcModel.initialVelocity;

            for i = 2:obj.d_config.N+1
              obj.d_initialStateGuess(obj.d_config.siIndex.s,i) =...
                obj.d_initialStateGuess(obj.d_config.siIndex.s,i - 1) + obj.d_ts * obj.d_parameters.mpcModel.initialVelocity;
              trackPosI = obj.d_track.centerLine.getPosition(obj.d_initialStateGuess(obj.d_config.siIndex.s,i));
              trackdPosI = obj.d_track.centerLine.getDerivative(obj.d_initialStateGuess(obj.d_config.siIndex.s,i));
              obj.d_initialStateGuess(obj.d_config.siIndex.x,i) = trackPosI(1);
              obj.d_initialStateGuess(obj.d_config.siIndex.y,i) = trackPosI(2);
              obj.d_initialStateGuess(obj.d_config.siIndex.yaw,i) = atan2(trackdPosI(2), trackdPosI(1));
              obj.d_initialStateGuess(obj.d_config.siIndex.vx,i) = obj.d_parameters.mpcModel.initialVelocity;
              obj.d_initialStateGuess(obj.d_config.siIndex.vs,i) = obj.d_parameters.mpcModel.initialVelocity;
            end
            obj.unwrapInitialGuess();
            obj.d_validInitialGuess = true;
        end

        function unwrapInitialGuess(obj)
            L = obj.d_track.centerLine.getLength();
            for i = 2:obj.d_config.N+1
              if (obj.d_initialStateGuess(obj.d_config.siIndex.yaw,i) - obj.d_initialStateGuess(obj.d_config.siIndex.yaw,i - 1)) < -pi
                obj.d_initialStateGuess(obj.d_config.siIndex.yaw,i) = obj.d_initialStateGuess(obj.d_config.siIndex.yaw,i) + 2.0 * pi;
              end
              if (obj.d_initialStateGuess(obj.d_config.siIndex.yaw,i) - obj.d_initialStateGuess(obj.d_config.siIndex.yaw,i - 1)) > pi
                obj.d_initialStateGuess(obj.d_config.siIndex.yaw,i) = obj.d_initialStateGuess(obj.d_config.siIndex.yaw,i) - 2.0 * pi;
              end
              if obj.d_initialStateGuess(obj.d_config.siIndex.s,i) >= L
                obj.d_initialStateGuess(obj.d_config.siIndex.s,i) = obj.d_initialStateGuess(obj.d_config.siIndex.s,i) - L;
              end
           end
        end

        function xNext = ode4(obj,state,input)
            % 4th order Runge Kutta (RK4) implementation
            % 4 evaluation points of continuous dynamics
            % evaluating the 4 points
            k1 = obj.f(state, input);
            k2 = obj.f(state + obj.d_ts / 2.0 * k1, input);
            k3 = obj.f(state + obj.d_ts / 2.0 * k2, input);
            k4 = obj.f(state + obj.d_ts * k3, input);
            % combining to give output
            xNext = state + obj.d_ts * (k1 / 6.0 + k2 / 3.0 + k3 / 3.0 + k4 / 6.0);
        end
    end
end


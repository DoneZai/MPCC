classdef KinematicSimulator < handle    
    properties (Access = private)
        d_car;
        d_tire;
        f;
    end
    
    methods (Access = public)

        function obj = KinematicSimulator(car,tire)
            import casadi.*;

            obj.d_car = car;
            obj.d_tire = tire;

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

            state = [x;y;yaw;vx;vy;r;s;throttle;steeringAngle;brakes;vs];

            dThrottle = SX.sym('dThrottle');
            dSteeringAngle = SX.sym('dSteeringAngle');
            dBrakes = SX.sym('dBrakes');
            dVs = SX.sym('dVs');

            input = [dThrottle,dSteeringAngle,dBrakes,dVs];

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

            obj.f = Function('f',{state,input},{rhs});
        end

        function xNext = ode4(obj,state,input,ts)
            % 4th order Runge Kutta (RK4) implementation
            % 4 evaluation points of continuous dynamics
            % evaluating the 4 points
            k1 = obj.f(state, input);
            k2 = obj.f(state + ts / 2.0 * k1, input);
            k3 = obj.f(state + ts / 2.0 * k2, input);
            k4 = obj.f(state + ts * k3, input);
            % combining to give output
            xNext = state + ts * (k1 / 6.0 + k2 / 3.0 + k3 / 3.0 + k4 / 6.0);
        end

        function xNext = simTimeStep(obj,x,u,ts)
            xNext = x;
            integrationSteps = cast(ts/0.001,'int64');
            
            for i = 1:integrationSteps
                xNext = obj.ode4(xNext,u,0.001).full();
            end
        end
    end
end


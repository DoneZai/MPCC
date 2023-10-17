classdef DynamicSimulator < handle    
    properties (Access = private)
        d_car;

        d_frontAxis;
        d_rearAxis;
        d_frontWheel;
        d_rearWheel;
    end
    
    methods (Access = public)

        function obj = DynamicSimulator(car,tire)
            obj.d_car = car;
            obj.d_frontWheel = Wheel(Side.FRONT,obj.d_car,tire);
            obj.d_rearWheel = Wheel(Side.REAR,obj.d_car,tire);
            obj.d_frontAxis = Axis(Side.FRONT,obj.d_car,obj.d_frontWheel);
            obj.d_rearAxis = Axis(Side.REAR,obj.d_car,obj.d_rearWheel);
        end

        function xNext = ode4(obj,state,input,ts)
            % 4th order Runge Kutta (RK4) implementation
            % 4 evaluation points of continuous dynamics
            % evaluating the 4 points
            k1 = obj.getF(vectorToState(state), vectorToInput(input));
            k2 = obj.getF(vectorToState(state + ts / 2.0 * k1), vectorToInput(input));
            k3 = obj.getF(vectorToState(state + ts / 2.0 * k2), vectorToInput(input));
            k4 = obj.getF(vectorToState(state + ts * k3), vectorToInput(input));
            % combining to give output
            xNext = state + ts * (k1 / 6.0 + k2 / 3.0 + k3 / 3.0 + k4 / 6.0);
        end

        function fdrag = getFdrag(obj,x)
            fdrag = obj.d_car.cd * x.vx^2.0;
        end

        function f = getF(obj,x,u)
            yaw = x.yaw;
            vx = x.vx;
            vy = x.vy;
            r = x.r;
            steeringAngle = x.steeringAngle;
            vs = x.vs;
            dBrakes = u.dBrakes;
            dThrottle = u.dThrottle;
            dSteeringAngle = u.dSteeringAngle;
            dVs = u.dVs;
            Ffx = obj.d_frontAxis.getFx(x);
            Ffy = obj.d_frontAxis.getFy(x);
            Frx = obj.d_rearAxis.getFx(x);
            Fry = obj.d_rearAxis.getFy(x);
            Fdrag = obj.getFdrag(x);

            f = zeros(11,1);

            f(1) = vx * cos(yaw) - vy * sin(yaw);
            f(2) = vx * sin(yaw) + vy * cos(yaw);
            f(3) = r;
            f(4) = 1 / obj.d_car.m *...
                   (Frx + cos(steeringAngle) * Ffx + Fdrag - sin(steeringAngle) * Ffy +...
                    obj.d_car.m * vy * r);
            f(5) = 1 / obj.d_car.m *...
                   (Fry + cos(steeringAngle) * Ffy + sin(steeringAngle) * Ffx - obj.d_car.m * vx * r);
            f(6) =...
              1 / obj.d_car.iz *...
              (-Fry * obj.d_car.lr + (cos(steeringAngle) * Ffy + sin(steeringAngle) * Ffx) * obj.d_car.lf);
            f(7) = vs;
            f(8) = dThrottle;
            f(9) = dSteeringAngle;
            f(10) = dBrakes;
            f(11) = dVs;
        end

        function xNext = simTimeStep(obj,x,u,ts)
            xNext = x;
            integrationSteps = cast(ts/0.001,'int64');
            
            for i = 1:integrationSteps
                xNext = obj.ode4(xNext,u,0.001);
            end
        end
    end
end


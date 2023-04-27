classdef Model
    %MODEL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        d_config
        d_ts
        d_car
        d_tire
        d_frontAxis
        d_rearAxis
        d_frontWheel
        d_rearWheel
    end
    
    methods (Access = public)
        function obj = Model(config,ts,car,tire)
            obj.d_config = config;
            obj.d_ts = ts;
            obj.d_car = car;
            obj.d_tire = tire;
            obj.d_frontWheel = Wheel(Side.FRONT,obj.d_car,obj.d_tire);
            obj.d_rearWheel = Wheel(Side.REAR,obj.d_car,obj.d_tire);
            obj.d_frontAxis = Axis(Side.FRONT,obj.d_car,obj.d_frontWheel);
            obj.d_rearAxis = Axis(Side.REAR,obj.d_car,obj.d_rearWheel);
        end

        function xNext = ode4(obj,state,input,ts)
            % 4th order Runge Kutta (RK4) implementation
            % 4 evaluation points of continuous dynamics
            xVec = stateToVector(state);
            % evaluating the 4 points
            k1 = obj.getF([],vectorToState(xVec), input);
            k2 = obj.getF([],vectorToState(xVec + ts / 2.0 * k1), input);
            k3 = obj.getF([],vectorToState(xVec + ts / 2.0 * k2), input);
            k4 = obj.getF([],vectorToState(xVec + ts * k3), input);
            % combining to give output
            xNext = xVec + ts * (k1 / 6.0 + k2 / 3.0 + k3 / 3.0 + k4 / 6.0);
        end

        function sa = getSaFront(obj,x)
            sa = obj.d_frontWheel.getSa(x);
        end

        function fdrag = getFdrag(obj,x)
            fdrag = obj.d_car.cd * x.vx^2.0;
        end
        
        function f = getF(obj,t,x,u)
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

        function dFdrag = getFdragDerivatives(obj,state)
            % Fdrag = cd * vx^2
            cd = obj.d_car.cd;
            vx = state.vx;
            dFdrag.dVx = cd * 2.0 * vx;
        end

        function dSa = getFrontSaDerivatives(obj,state)
            %sa = atan2((vy + r * obj.d_car.lf),vx) - steeringAngle;
            lf = obj.d_car.lf;
            vx = state.vx;
            vy = state.vy;
            r = state.r; 

            dSa.dVx = -(lf * r + vy) / (vx^2.0 + (lf * r + vy)^2.0);
            dSa.dVy = vx / (vx^2.0 + (lf * r + vy)^2.0);
            dSa.dR = lf * vx / (vx^2.0 + (lf * r + vy)^2.0);
            dSa.dSteeringAngle = -1.0;
        end

        function dSa = getRearSaDerivatives(obj,state)
            %sa = atan2((vy - r * obj.d_car.lr),vx);
            lr = obj.d_car.lr;
            vx = state.vx;
            vy = state.vy;
            r = state.r; 

            dSa.dVx = (lr * r - vy) / (vx^2.0 + (-lr * r + vy)^2.0);
            dSa.dVy = vx / (vx^2.0 + (-lr * r + vy)^2.0);
            dSa.dR = -lr * vx / (vx^2.0 + (-lr * r + vy)^2.0);
            dSa.dSteeringAngle = 0.0;
        end



        function dFfx = getFrontFxDerivatives(obj)
            % Ffx = - cbf * brakes / rDyn - Frr
            cbf = obj.d_car.cbf;
            rDyn = obj.d_car.rDyn;
              
            dFfx.dThrottle = 0.0;
            dFfx.dBrakes = -cbf / rDyn;
        end

        function dFrx = getRearFxDerivatives(obj)
            % Frx = cdrivetrain * throttle / rDyn - cbr * brakes / rDyn - Frr
            cbr = obj.d_car.cbr;
            rDyn = obj.d_car.rDyn;
            cdrivetrain = obj.d_car.cm1 * obj.d_car.gearRatio;
              
            dFrx.dThrottle = cdrivetrain / rDyn;
            dFrx.dBrakes = -cbr / rDyn;
        end

        function dFfy = getFrontFyDerivatives(obj,state)
            % Ffy = 2 * Dy * sin(Cy * atan(By * sa - Ey * (By * sa - atan(By * sa))))
            % Ffy = 2 * Dy * sin(m),
            % m = Cy * atan(n),
            % n = By * sa - Ey * (By * sa - atan(By * sa))
            sa = obj.d_frontWheel.getSa(state);
            Ky = obj.d_frontWheel.getKy();
            Dy = obj.d_frontWheel.getDy();
            Cy = obj.d_frontWheel.getCy();
            Ey = obj.d_frontWheel.getEy();
            By = Ky / (Cy * Dy);
            n = By * sa - Ey * (By * sa - atan(By * sa));
            m = Cy * atan(n);
            dNdSa = By - Ey * By + Ey / (1.0 + (By * sa)^2.0) * By;
            dMdN = Cy / (1.0 + n^2.0);
            dFydM = 2 * Dy * cos(m);
            dFydSa = dFydM * dMdN * dNdSa;
            dSa = obj.getFrontSaDerivatives(state);
            dFfy.dVx = dFydSa * dSa.dVx;
            dFfy.dVy = dFydSa * dSa.dVy;
            dFfy.dR = dFydSa * dSa.dR;
            dFfy.dSteeringAngle = dFydSa * dSa.dSteeringAngle;
        end

        function dFry = getRearFyDerivatives(obj,state)
            % Fry = 2 * Dy * sin(Cy * atan(By * sa - Ey * (By * sa - atan(By * sa))))
            % Fry = 2 * Dy * sin(m),
            % m = Cy * atan(n),
            % n = By * sa - Ey * (By * sa - atan(By * sa))
            sa = obj.d_rearWheel.getSa(state);
            Ky = obj.d_rearWheel.getKy();
            Dy = obj.d_rearWheel.getDy();
            Cy = obj.d_rearWheel.getCy();
            Ey = obj.d_rearWheel.getEy();
            By = Ky / (Cy * Dy);
            n = By * sa - Ey * (By * sa - atan(By * sa));
            m = Cy * atan(n);
            dNdSa = By - Ey * By + Ey / (1.0 + (By * sa)^2.0) * By;
            dMdN = Cy / (1.0 + n^2.0);
            dFydM = 2 * Dy * cos(m);
            dFydSa = dFydM * dMdN * dNdSa;
            dSa = obj.getRearSaDerivatives(state);
            dFry.dVx = dFydSa * dSa.dVx;
            dFry.dVy = dFydSa * dSa.dVy;
            dFry.dR = dFydSa * dSa.dR;
            dFry.dSteeringAngle = 0.0;
        end

        function linModelC = getModelJacobianMatlab(obj,state,input)
            linModelC = LinModelMatrix();

            m = obj.d_car.m;
            iz = obj.d_car.iz;
            lf = obj.d_car.lf;
            lr = obj.d_car.lr;
            cbf = obj.d_car.cbf;
            cbr = obj.d_car.cbr;
            cm1 = obj.d_car.cm1;
            gearRatio = obj.d_car.gearRatio;
            rDyn = obj.d_car.rDyn;
            
            x = state.x;
            y = state.y;
            yaw = state.yaw; 
            vx = state.vx;
            vy = state.vy;
            r = state.r;
            s = state.s;
            throttle = state.throttle;
            steeringAngle = state.steeringAngle;
            brakes = state.brakes;
            vs = state.vs;

            syms X Y Yaw Vx Vy R S Throttle SteeringAngle Brakes Vs;

            Dyf = obj.d_frontWheel.getDy();
            Cyf = obj.d_frontWheel.getCy();
            Kyf = obj.d_frontWheel.getKy();
            Byf = Kyf / (Cyf * Dyf);
            Eyf = obj.d_frontWheel.getEy();

            Ffrr = 2 * obj.d_frontWheel.getFrr();

            Ffx = -cbf * Brakes / rDyn + Ffrr;
            af = atan2((Vy + R * lf),Vx) - SteeringAngle;
            Ffy = Dyf * sin(Cyf * atan(Byf * af - Eyf * (Byf * af - atan(Byf * af))));

            Dyr = obj.d_rearWheel.getDy();
            Cyr = obj.d_rearWheel.getCy();
            Kyr = obj.d_rearWheel.getKy();
            Byr = Kyr / (Cyr * Dyr);
            Eyr = obj.d_rearWheel.getEy();
            
            Frrr = 2 * obj.d_rearWheel.getFrr();
            
            Frx = cm1 * gearRatio * Throttle / rDyn -...
                cbr * Brakes / rDyn + Frrr;
            ar = atan2((Vy - R * obj.d_car.lr),Vx);
            Fry = Dyr * sin(Cyr * atan(Byr * ar - Eyr * (Byr * ar - atan(Byr * ar))));

            Fdrag = obj.getFdrag(state);

            Ac = jacobian([ ...
                Vx * cos(Yaw) - Vy * sin(Yaw), ...
                Vx * sin(Yaw) + Vy * cos(Yaw), ...
                R ...
                1 / m *...
                   (Frx + cos(steeringAngle) * Ffx + Fdrag - sin(steeringAngle) * Ffy +...
                    m * vy * r), ...
                1 / m *...
                   (Fry + cos(steeringAngle) * Ffy + sin(steeringAngle) * Ffx - m * vx * r), ...
                1 / iz *...
                (-Fry * lr + (cos(steeringAngle) * Ffy + sin(steeringAngle) * Ffx) * lf), ...
                Vs, ...
                0.0, ...
                0.0, ...
                0.0, ...
                0.0],[X,Y,Yaw,Vx,Vy,R,S,Throttle,SteeringAngle,Brakes,Vs]);
            
            linModelC.a = double(subs(Ac,{X,Y,Yaw,Vx,Vy,R,S,Throttle,SteeringAngle,Brakes,Vs}, ...
                {x,y,yaw,vx,vy,r,s,throttle,steeringAngle,brakes,vs}));

            linModelC.b = zeros(11,4);
            linModelC.b(8:11,1:4) = eye(4);

            f = obj.getF([],state,input);

            linModelC.g = f - linModelC.a * stateToVector(state) - linModelC.b * inputToVector(input);
        end

        function linModelC = getModelJacobian(obj,state,input)
            Ac = zeros(obj.d_config.NX,obj.d_config.NX);
            Bc = zeros(obj.d_config.NU,obj.d_config.NU);

            m = obj.d_car.m;
            iz = obj.d_car.iz;
            lf = obj.d_car.lf;
            lr = obj.d_car.lr;

            yaw = state.yaw;
            vx = state.vx;
            vy = state.vy;
            r = state.r;
            steeringAngle = state.steeringAngle;
           
            % F1 = vx*cos(yaw) - vy*sin(yaw)
            dF1dYaw = vx * (-sin(yaw)) - vy * cos(yaw);
            dF1dVx = cos(yaw);
            dF1dVy = -sin(yaw);

            % F2 = vy*cos(yaw) + vx*sin(yaw);
            dF2dYaw = vy * (-sin(yaw)) + vx * cos(yaw);
            dF2dVx = sin(yaw);
            dF2dVy = cos(yaw);

            % F3 = r;
            dF3dR = 1.0;
           
            dFfx = obj.getFrontFxDerivatives();
            dFfy = obj.getFrontFyDerivatives(state);
            dFrx = obj.getRearFxDerivatives();
            dFry = obj.getRearFyDerivatives(state);

            dFdrag = obj.getFdragDerivatives(state);

            Ffx = obj.d_frontAxis.getFx(state);
            Ffy = obj.d_frontAxis.getFy(state);
            
            % F4 = 1 / m * (Frx + cos(steeringAngle) * Ffx + Fdrag - sin(steeringAngle) * Ffy + m * vy * r)
            dF4dVx = 1.0 / m * (dFdrag.dVx - sin(steeringAngle) * dFfy.dVx);
            dF4dVy = 1.0 / m * (-sin(steeringAngle) * dFfy.dVy + m * r);
            dF4dR = 1.0 / m * (-sin(steeringAngle) * dFfy.dR + m * vy);
            dF4dThrottle = 1.0 / m * (dFrx.dThrottle);
            dF4dSteeringAngle =...
              1.0 / m *...
              (-sin(steeringAngle) * Ffx - cos(steeringAngle) * Ffy -...
               sin(steeringAngle) * dFfy.dSteeringAngle);
            dF4dBrakes = 1.0 / m * (dFrx.dBrakes + cos(steeringAngle) * dFfx.dBrakes); 

            % F5 = 1 / m * (Fry + cos(steeringAngle) * Ffy + sin(steeringAngle) * Ffx - m * vx * r)
            dF5dVx =...
                1.0 / m * (dFry.dVx + cos(steeringAngle) * dFfy.dVx - m * r);
            dF5dVy = 1.0 / m * (dFry.dVy + cos(steeringAngle) * dFfy.dVy);
            dF5dR =...
                1.0 / m * (dFry.dR + cos(steeringAngle) * dFfy.dR - m * vx);
            dF5dSteeringAngle =...
                1.0 / m *...
                (dFry.dSteeringAngle - sin(steeringAngle) * Ffy +...
                 cos(steeringAngle) * dFfy.dSteeringAngle + cos(steeringAngle) * Ffx);
            
            dF5dBrakes = 1 / m * sin(steeringAngle) * dFfx.dBrakes;
            
            % F6 = 1 / I * (-Fry * lr + (std::cos(steeringAngle) * Ffy + std::sin(steeringAngle) * Ffx) * lf)
            dF6dVx =...
                1.0 / iz *...
                (-dFry.dVx * lr + (cos(steeringAngle) * dFfy.dVx) * lf);
            dF6dVy =...
                1.0 / iz *...
                (-dFry.dVy * lr + (cos(steeringAngle) * dFfy.dVy) * lf);
            dF6dR =...
                1.0 / iz *...
                (-dFry.dR * lr + (cos(steeringAngle) * dFfy.dR) * lf);
            dF6dSteeringAngle =...
                1.0 / iz *...
                (-dFry.dSteeringAngle * lr +...
                 (-sin(steeringAngle) * Ffy + cos(steeringAngle) * dFfy.dSteeringAngle +...
                  cos(steeringAngle) * Ffx) *...
                   lf);
            dF6dBrakes = 1.0 / iz * sin(steeringAngle) * dFfx.dBrakes * lf;

            % Jacobians
            % Matrix A
            % Column 1
            % all zero
            % Column 2
            % all zero
            % Column 3
            Ac(1, 3) = dF1dYaw;
            Ac(2, 3) = dF2dYaw;
            % Column 4
            Ac(1, 4) = dF1dVx;
            Ac(2, 4) = dF2dVx;
            Ac(4, 4) = dF4dVx;
            Ac(5, 4) = dF5dVx;
            Ac(6, 4) = dF6dVx;
            % Column 5
            Ac(1, 5) = dF1dVy;
            Ac(2, 5) = dF2dVy;
            Ac(4, 5) = dF4dVy;
            Ac(5, 5) = dF5dVy;
            Ac(6, 5) = dF6dVy;
            % Column 6
            Ac(3, 6) = dF3dR;
            Ac(4, 6) = dF4dR;
            Ac(5, 6) = dF5dR;
            Ac(6, 6) = dF6dR;
            % Column 7
            % all zero
            % Column 8
            Ac(4, 8) = dF4dThrottle;
            % Column 9
            Ac(4, 9) = dF4dSteeringAngle;
            Ac(5, 9) = dF5dSteeringAngle;
            Ac(6, 9) = dF6dSteeringAngle;
            % Column 10
            Ac(4, 10) = dF4dBrakes;
            Ac(5, 10) = dF5dBrakes;
            Ac(6, 10) = dF6dBrakes;
            % Column 11
            Ac(7, 11) = 1.0;
            % Matrix B
            % Column 1
            Bc(8, 1) = 1.0;
            % Column 2
            Bc(9, 2) = 1.0;
            % Column 3
            Bc(10, 3) = 1.0;
            % Column 4
            Bc(11, 4) = 1.0;
            f = obj.getF([],state,input);
            % zero order term
            gc = f - Ac * stateToVector(state) - Bc * inputToVector(input);
            linModelC = LinModelMatrix(Ac,Bc,gc);
        end

        function model = discretizeModel(obj,linModelC,x,u,xk1Nz)
            temp = zeros(obj.d_config.NX + obj.d_config.NU,obj.d_config.NX + obj.d_config.NU);
            temp(1:obj.d_config.NX,1:obj.d_config.NX) = linModelC.a;
            temp(1:obj.d_config.NX,1+obj.d_config.NX:obj.d_config.NX+obj.d_config.NU) = linModelC.b;
            temp = temp * obj.d_ts;

            temp_res = expm(temp);
            
            Ad = temp_res(1:obj.d_config.NX,1:obj.d_config.NX);
            Bd = temp_res(1:obj.d_config.NX,1+obj.d_config.NX:obj.d_config.NX+obj.d_config.NU);
            
            %x0 = stateToVector(x);
            %opts = odeset('RelTol',1e+2,'AbsTol',1e+2,'InitialStep',obj.d_ts,'MaxStep',obj.d_ts);
            %[~,inivt]=ode45(@(t,inivt)obj.getF(t,x,u),[0 obj.d_ts],x0,opts);
            %xRk = inivt(end,:)';
            xRk = obj.ode4(x,u,obj.d_ts);
            
            gD = -stateToVector(xk1Nz) + xRk;
            model = LinModelMatrix(Ad,Bd,gD);
        end

        function model = getLinModel(obj,x,u,xk1Nz)
            linModelC = obj.getModelJacobian(x,u);
            model = obj.discretizeModel(linModelC, x, u, xk1Nz);
        end
    end
end


classdef Wheel < handle
    %WHEEL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        d_side
        d_car
        d_tire
        d_Fz
        d_Dfz
    end
    
    methods
        function obj = Wheel(side,car,tire)
            obj.d_side = side;
            obj.d_car = car;
            obj.d_tire = tire;
            
            if obj.d_side == Side.FRONT
                obj.d_Fz = car.lr * car.m * car.g /...
                          (2.0 * (car.lf + car.lr));
            else
                obj.d_Fz = car.lf * car.m * car.g /...
                          (2.0 * (car.lf + car.lr));
            end

            obj.d_Dfz = (obj.d_Fz - car.fzNominal) / car.fzNominal;  
        end
        
        function sa = getSa(obj,state)
            vx = state.vx;
            vy = state.vy;
            r = state.r;
            steeringAngle = state.steeringAngle;
            lf = obj.d_car.lf;
            lr = obj.d_car.lr;
            if obj.d_side == Side.FRONT
              sa = atan2((vy + r * lf), vx) - steeringAngle;
            else
              sa = atan2((vy - r * lr), vx);
            end
        end

        function sign = getSign(obj,var)
            if var >= 0.0
                sign = 1.0;
            else
                sign = -1.0;
            end
        end

        function frr = getFrr(obj)
            frr = obj.d_tire.QSY1 * obj.d_Fz;
        end

        function ky = getKy(obj)
            ky = obj.d_tire.PKY1 * obj.d_car.fzNominal *...
            sin(2.0 * atan2(obj.d_Fz, (obj.d_tire.PKY2 * obj.d_car.fzNominal * obj.d_tire.LFZO))) * obj.d_tire.LFZO *...
            obj.d_tire.LKY;
        end

        function dy = getDy(obj)
            muy = (obj.d_tire.PDY1 + obj.d_tire.PDY2 * obj.d_Dfz) * obj.d_tire.LMUY;
            dy = muy * obj.d_Fz;
        end

        function cy = getCy(obj)
            cy = obj.d_tire.PCY1 * obj.d_tire.LCY;
        end

        %function shy = getShy(obj)
        %    shy = (obj.d_tire.PHY1 + obj.d_tire.PHY2 * obj.d_Dfz) * obj.d_tire.LHY;
        %end

        function ey = getEy(obj)
            ey = (obj.d_tire.PEY1 + obj.d_tire.PEY2 * obj.d_Dfz) * obj.d_tire.LEY;
        end

        function fy = getFy(obj,state)
            a = obj.getSa(state);
            Ky = obj.getKy();
            Dy = obj.getDy();
            Cy = obj.getCy();
            By = Ky / (Cy * Dy);
            Ey = obj.getEy();
              
            fy = Dy * sin(Cy * atan(By * a - Ey * (By * a - atan(By * a))));
        end
    end
end


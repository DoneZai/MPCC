classdef Axis
    properties
        d_side
        d_car
        d_wheel
    end
    
    methods (Access = public)

        function obj = Axis(side,car,wheel)
            obj.d_side = side;
            obj.d_car = car;
            obj.d_wheel = wheel;
        end

        function fx = getFx(obj,state)
            if obj.d_side == Side.FRONT
                cb = obj.d_car.cbf;
                cdrv = obj.d_car.cm1 * obj.d_car.gearRatio;
            else
                cb = obj.d_car.cbr;
                cdrv = 0.0;
            end
            fbr = -cb * state.brakes / obj.d_car.rDyn;
            fdrv = cdrv * state.throttle / obj.d_car.rDyn;
            frr = 2.0 * obj.d_wheel.getFrr();
            fx = fbr + fdrv + frr;
        end

        function fy = getFy(obj,state)
            fy = 2 * obj.d_wheel.getFy(state);
        end
    end
end


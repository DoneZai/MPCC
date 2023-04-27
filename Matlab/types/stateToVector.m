function xk = stateToVector(x)
    xk = zeros(11,1);
    xk(1) = x.x;
    xk(2) = x.y;
    xk(3) = x.yaw;
    xk(4) = x.vx;
    xk(5) = x.vy;
    xk(6) = x.r;
    xk(7) = x.s;
    xk(8) = x.throttle;
    xk(9) = x.steeringAngle;
    xk(10) = x.brakes;
    xk(11) = x.vs;
end

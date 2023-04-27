function uk = inputToVector(u)
    uk = zeros(4,1);
    uk(1) = u.dThrottle;
    uk(2) = u.dSteeringAngle;
    uk(3) = u.dBrakes;
    uk(4) = u.dVs;
end


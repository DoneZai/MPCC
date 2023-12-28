m = parameters.car.m;
gAcc = parameters.car.g;
fzNominal = parameters.car.fzNominal;
cbr=parameters.car.cbr;
rDyn = parameters.car.rDyn;
cdrv = parameters.car.cm1 * parameters.car.gearRatio;

Frz = lf*m*gAcc/(2.0*(lf+lr));
Drfz = (Frz-fzNominal)/fzNominal;
Kry = parameters.tire.PKY1*fzNominal * sin(2.0*atan2(Frz,(parameters.tire.PKY2*fzNominal*parameters.tire.LFZO)))*parameters.tire.LFZO*parameters.tire.LKY;
Dry = (parameters.tire.PDY1+parameters.tire.PDY2*Drfz)*parameters.tire.LMUY*Frz;
Cry = parameters.tire.PCY1*parameters.tire.LCY;
Bry = Kry/(Cry*Dry);
Ery = (parameters.tire.PEY1+parameters.tire.PEY2*Drfz)*parameters.tire.LEY;
sar = atan2((vy-r*lr),vx);

Frx = (-cbr*brakes)/rDyn*tanh(vx)+(cdrv*throttle)/rDyn+2*parameters.tire.QSY1*Frz*tanh(vx);
Fry = 2*Dry*sin(Cry*atan(Bry*sar-Ery*(Bry*sar-atan(Bry*sar))));

Ffz = lr*m*gAcc/(2.0*(lf+lr));
Dffz = (Ffz-fzNominal)/fzNominal;
Kfy = parameters.tire.PKY1*fzNominal * sin(2.0*atan2(Ffz,(parameters.tire.PKY2*fzNominal*parameters.tire.LFZO)))*parameters.tire.LFZO*parameters.tire.LKY;        
mufy = (parameters.tire.PDY1+parameters.tire.PDY2*Dffz)*parameters.tire.LMUY;
Dfy = mufy*Ffz;
Cfy = parameters.tire.PCY1*parameters.tire.LCY;
Bfy = Kfy/(Cfy*Dfy);
Efy = (parameters.tire.PEY1+parameters.tire.PEY2*Dffz)*parameters.tire.LEY;
saf = atan2((vy+r*lf),vx)-steeringAngle;

Ffy = 2*Dfy*sin(Cfy*atan(Bfy*saf-Efy*(Bfy*saf-atan(Bfy*saf))));
Ffx = (-cbf*brakes)/rDyn*tanh(vx)+2*parameters.tire.QSY1*Ffz*tanh(vx);

% muy*Fz=2030.5; %a
% mux*Fz=2118.04; %b

(Ffx/2118.04)^2+(Ffy/2030.5)^2<1;
(Frx/2118.04)^2+(Fry/2030.5)^2<1;

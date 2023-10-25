function model = getModel(parameters)
    import casadi.*;

    % State
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

    % Controls
    dThrottle = SX.sym('dThrottle');
    dSteeringAngle = SX.sym('dSteeringAngle');
    dBrakes = SX.sym('dBrakes');
    dVs = SX.sym('dVs');

    input = [dThrottle;dSteeringAngle;dBrakes;dVs];

    % xdot
    xDot = SX.sym('xDot');
    yDot = SX.sym('yDot');
    yawDot = SX.sym('yawDot');
    vxDot = SX.sym('vxDot');
    vyDot = SX.sym('vyDot');
    rDot = SX.sym('rDot');
    sDot = SX.sym('sDot');
    throttleDot = SX.sym('throttleDot');
    steeringAngleDot = SX.sym('steeringAngleDot');
    brakesDot = SX.sym('brakesDot');
    vsDot = SX.sym('vsDot');

    xdot = [xDot;yDot;yawDot;vxDot;vyDot;rDot;sDot;throttleDot;steeringAngleDot;brakesDot;vsDot];
    
    % algebraic variables
    z = [];

    % parameters
    xCen = SX.sym('xCen');
    yCen = SX.sym('yCen');
    xRef = SX.sym('xRef');
    yRef = SX.sym('yRef');
    thetaRef = SX.sym('thetaRef');

    p = [xCen;yCen;xRef;yRef;thetaRef];
    
    % dynamics
    carModel = Model(parameters.car,parameters.tire);
    f_expl = carModel.initSimpleCombinedModel(state,input);
    f_impl = f_expl - xdot;

    % cost
    % contouring error
    ec = -sin(thetaRef) * (xRef - x)...
                            + cos(thetaRef) * (yRef - y);
    % lag error
    el = cos(thetaRef) * (xRef - x)...
                            + sin(thetaRef) * (yRef - y);
    
    error = [ec;el];

    % Coeffs for laf and contouring errors penallization
    Q = diag([parameters.costs.qC, ...
              parameters.costs.qL]);

    q = parameters.costs.qVs;

    % Coeffs for control inputs penalization
    R = diag([parameters.costs.rThrottle, ...
              parameters.costs.rSteeringAngle, ...
              parameters.costs.rBrakes, ...
              parameters.costs.rVs]);

    cost_expr_ext_cost = error' * Q * error + input'*R*input - q*vs;

    % constraints 
    lf = parameters.car.lf;
    lr = parameters.car.lr;

    constr_expr_h = [(x-xCen)^2 + (y-yCen)^2, ...
                   atan2((vy + r*lf),vx) - steeringAngle, ...
                   atan2((vy - r*lr),vx)];

    % model filling
    model.f_expl_expr = f_expl;
    model.f_impl_expr = f_impl;
    model.x = state;
    model.xdot = xdot;
    model.u = input;
    model.p = p;
    model.z = z;
    model.cost_expr_ext_cost = cost_expr_ext_cost;
    model.constr_expr_h = constr_expr_h;
end


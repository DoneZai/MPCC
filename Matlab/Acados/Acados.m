classdef Acados < handle
    %ACADOS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        config
        parameters
        carModel

        track

        ocpModel
        ocpOpts
        ocp

        p % parameters vector: [Xcen, Ycen, xRef, yRef, thetaRef];
    end
    
    methods (Access = public)
        function obj = Acados(config,parameters)
            check_acados_requirements();

            obj.config = config;
            obj.parameters = parameters;
            obj.carModel = Model(parameters.car,parameters.tire);

            obj.ocpModel = acados_obj.ocpModel();
            obj.ocpOpts = acados_obj.ocpOpts();
        end

        function setTrack(obj,track)
            import casadi.*;

            obj.track.centerLine.gen2DSpline([track.x;track.x],[track.y;track.y]);
            obj.track.outerBorder.gen2DSpline([track.xOuter;track.xOuter],[track.yOuter,track.yOuter]);
            obj.track.innerBorder.gen2DSpline([track.xInner;track.xInner],[track.yInner,track.yInner]);

            centerLine = obj.track.centerLine.getPath();
            outerBoredr = obj.track.outerBorder.getPath();
            innerBorder = obj.track.innerBorder.getPath();

            centerLineDerivatives = zeros(2,length(centerLine.s));
            
            for i = 1:length(centerLine.s)
                centerLineDerivatives(:,i) = obj.track.centerLine.getDerivative(centerLine.s(i));
            end
            
            obj.track.centerLineInterpolation.x = interpolant('center_line_interpolation_x','bspline',{centerLine.s},centerLine.x);
            obj.track.centerLineInterpolation.y = interpolant('center_line_interpolation_y','bspline',{centerLine.s},centerLine.y);
            obj.track.centerLineDerivativesInterpolation.x = interpolant('center_line_derivative_interpolation_x','bspline',{centerLine.s},centerLineDerivatives(1,:));
            obj.track.centerLineDerivativesInterpolation.y = interpolant('center_line_derivative_interpolation_y','bspline',{centerLine.s},centerLineDerivatives(2,:));

            obj.track.outerBorderInterpolation.x = interpolant('outerBorder_interpolation_x','bspline',{outerBoredr.s},outerBoredr.x);
            obj.track.outerBorderInterpolation.y = interpolant('outerBorder_interpolation_y','bspline',{outerBoredr.s},outerBoredr.y);

            obj.track.innerBorderInterpolation.x = interpolant('innerBorder_interpolation_x','bspline',{innerBorder.s},innerBorder.x);
            obj.track.innerBorderInterpolation.y = interpolant('innerBorder_interpolation_y','bspline',{innerBorder.s},innerBorder.y);
        end

        function initMPC(obj)
            obj.initOcpModel();
            obj.initConstraints();
            obj.setOCPOpts();
        end

        function initOcpModel(obj)
            obj.ocpModel.set('name','acados_mpcc');
            obj.ocpModel.set('T', obj.config.N*obj.parameters.ts);

            import casadi.*`
            
            % States
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

            input = [dThrottle,dSteeringAngle,dBrakes,dVs];
            
            % Rate of stete change
            xdot = SX.sym('xdot', obj.config.NX, 1);

            % Dynamics
            rhs = obj.carModel.initSimpleCombinedModel(sym_x,sym_u);

            obj.ocpModel.set('sym_x',state);
            obj.ocpModel.set('sym_u',input);
            obj.ocpModel.set('sym_xdot',xdot);
            obj.ocpModel.set('dyn_type','explicit');
            obj.ocpModel.set('dyn_expr_f',rhs);

            obj.p = SX.sym('p',obj.config.N*3 + (obj.config.N+1)*2,1); % parameters vector [Xcen, Ycen, xRef, yRef, thetaRef];
        end

        function initConstraints()
            x = 
            constr_expr_h = 
            obj.ocpModel.set('constr_expr_h',);

            
        end

        function setBounds(obj)
            nbx = 7;
            jbx = zeros(nbx,obj.config.NX);

            jbx(1,3) = 1;
            jbx(2,4) = 1;
            jbx(3,7) = 1;
            jbx(4,8) = 1;
            jbx(5,9) = 1;
            jbx(6,10) = 1;
            jbx(7,11) = 1;

            obj.ocpModel.set('constr_Jbx',jbx);

            obj.ocpModel.set('constr_lbx',[ ...
                                            -2*pi, ...
                                            1.0, ...
                                            0, ...
                                            obj.parameters.bounds.lowerStateBounds.throttleL, ...
                                            obj.parameters.bounds.lowerStateBounds.steeringAngleL, ...
                                            obj.parameters.bounds.lowerStateBounds.brakesL, ...
                                            0]);

            obj.ocpModel.set('constr_ubx',[ ...
                                            2*pi, ...
                                            15, ...
                                            10e8, ...
                                            obj.parameters.bounds.upperStateBounds.throttleU, ...
                                            obj.parameters.bounds.upperStateBounds.steeringAngleU, ...
                                            obj.parameters.bounds.upperStateBounds.brakesU, ...
                                            10e8]);

            nbu = 3;
            jbu = zeros(nbu,obj.config.NU);

            jbu(1,1) = 1;
            jbu(2,2) = 1;
            jbu(3,3) = 1;

            obj.ocpModel.set('constr_Jbu',jbu);

            obj.ocpModel.set('constr_lbu',[ ...
                                            obj.parameters.bounds.lowerInputBounds.dThrottleL, ...
                                            obj.parameters.bounds.lowerInputBounds.dSteeringAngleL, ...
                                            obj.parameters.bounds.lowerInputBounds.dBrakesL]);

            obj.ocpModel.set('constr_ubu',[ ...
                                            obj.parameters.bounds.upperInputBounds.dThrottleU, ...
                                            obj.parameters.bounds.upperInputBounds.dSteeringAngleU, ...
                                            obj.parameters.bounds.upperInputBounds.dBrakesU]);

        end

        function setEqualityConstraint(obj,x0)
            obj.ocp.set('constr_x0', x0);
            obj.ocp.set('constr_lbx', x0, 0);
            obj.ocp.set('constr_ubx', x0, 0);
        end

        function setOCPOpts(obj)
            obj.ocpOpts.set('param_scheme_N', obj.config.N);
            obj.ocpOpts.set('nlp_solver', 'sqp_rti'); % sqp, sqp_rti 
            obj.ocpOpts.set('nlp_solver_exact_hessian', 'false'); % false=gauss_newton, true=exact
            obj.ocpOpts.set('sim_method', 'erk'); % erk, irk, irk_gnsf
            obj.ocpOpts.set('sim_method_num_stages', 4);
            obj.ocpOpts.set('sim_method_num_steps', 3);
            obj.ocpOpts.set('qp_solver', 'partial_condensing_hpipm');
            obj.ocpOpts.set('qp_solver_cond_N', 5);
            obj.ocpOpts.set('nlp_solver_tol_stat', 1e-4);
            obj.ocpOpts.set('nlp_solver_tol_eq', 1e-4);
            obj.ocpOpts.set('nlp_solver_tol_ineq', 1e-4);
            obj.ocpOpts.set('nlp_solver_tol_comp', 1e-4);

            obj.ocp = acados_ocp(obj.ocpModel, obj.ocpOpts);
        end


        

    end
end


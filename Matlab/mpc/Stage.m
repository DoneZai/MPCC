classdef Stage
    properties (Access = public)
        linModel
        costMat
        constrainsMat
        uBoundsX
        lBoundsX
        uBoundsU
        lBoundsU
        uBoundsS
        lBoundsS
        nx % number of states
        nu % number of inputs
        nbx % number of bounds on x
        nbu % number of bounds on u
        ng % number of polytopic constratins
        ns % number of soft constraints
    end

    properties (Access = private)
        d_config
    end

    methods (Access = public)
        function obj = Stage(config)
            obj.d_config = config;
            obj.linModel = LinModelMatrix();
            obj.costMat = CostMatrix();
            obj.constrainsMat = ConstraintsMatrix();
            obj.uBoundsX = zeros(obj.d_config.NX,1);
            obj.lBoundsX = zeros(obj.d_config.NX,1);
            obj.uBoundsU = zeros(obj.d_config.NU,1);
            obj.lBoundsU = zeros(obj.d_config.NU,1);
            obj.uBoundsS = zeros(obj.d_config.NS,1);
            obj.lBoundsS = zeros(obj.d_config.NS,1);
        end
    end
end

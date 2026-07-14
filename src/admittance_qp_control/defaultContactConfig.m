function cfg = defaultContactConfig()
%DEFAULTCONTACTCONFIG Default UR5e virtual-contact experiment parameters.
%
% The joint-angle limits in this file are simulation software limits used
% by the QP. They are not claimed to be certified manufacturer limits.

    cfg = struct;

    %% Robot and tool
    cfg.robotType = 'ur5e';
    cfg.q0 = [0; -pi/2; pi/2; -pi/2; -pi/2; 0];

    cfg.gToolPen = [1, 0, 0, 0;
                    0, 1, 0, -0.049;
                    0, 0, 1, 0.12228;
                    0, 0, 0, 1];

    %% Simulation timing
    cfg.dt = 0.005;

    cfg.approachTime = 1.5;
    cfg.pushTime = 1.0;
    cfg.holdTime = 1.5;
    cfg.returnTime = 1.5;
    cfg.settleTime = 1.0;

    %% Nominal Cartesian contact trajectory
    % Plane normal points from free space into the virtual obstacle.
    cfg.contactNormal = [0; 1; 0];
    cfg.surfaceDistance = 0.006;  % plane is 6 mm from the initial pen tip
    cfg.pushDepth = 0.004;        % nominal trajectory pushes 4 mm past plane

    %% Kelvin-Voigt virtual environment
    cfg.environmentStiffness = 3000; % N/m
    cfg.environmentDamping = 30;     % N*s/m

    %% Cartesian admittance model
    % M_d xdd_a + D_d xd_a + K_d x_a = F_ext
    cfg.admittanceMass = diag([2, 2, 2]);
    cfg.admittanceDamping = diag([80, 80, 80]);
    cfg.admittanceStiffness = diag([800, 800, 800]);

    %% Cartesian feedback
    cfg.Kp = 8.0;
    cfg.Kr = 4.0;

    %% Velocity-level QP
    cfg.taskWeight = diag([1, 1, 1, 0.2, 0.2, 0.2]);
    cfg.velocityRegularization = 1e-4;
    cfg.smoothnessWeight = 2e-3;

    cfg.jointSpeedMax = 0.23 * ones(6,1);

    % Conservative software angle envelope for the offline simulation.
    cfg.jointMin = -2*pi * ones(6,1);
    cfg.jointMax =  2*pi * ones(6,1);
    cfg.jointMargin = deg2rad(2) * ones(6,1);

    %% Recovery metrics
    cfg.forceThreshold = 0.1;          % N
    cfg.positionTolerance = 0.0005;    % 0.5 mm
    cfg.recoveryDwellTime = 0.10;      % s
end

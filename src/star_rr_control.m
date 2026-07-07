%% star_rr_control.m
%% Using Resolve-rate Control to make ur5(e) to draw a five-pointed star

clc;
clear;

%% Initialize robot interface
ur = ur_interface();
pause(0.5);

%% --- 1. Parameter Settings ---
R = 0.02;          % 五角星半径
T_side = 5;        % 每条边时间
dt = 0.15;

type = 'ur5e';% 'ur5' or 'ur5e'
operating_mode = 'RVIZ'; % 'RVIZ' or 'real ur5(e)'

%% Pen tip offset
d_z = 0.12228;
d_y = -0.049;
g_t_p = [1 0 0   0;
         0 1 0 d_y;
         0 0 1 d_z;
         0 0 0   1];

%% --- 2. Acquire Start Pose ---
if strcmp(operating_mode,'RVIZ')
    g_st1 = [ 0, -1,  0,  0.25;
             -1,  0,  0,  0.60;
              0,  0, -1,  0.22;
              0,  0,  0,     1];
    g_st2 = [ 0, -1,  0,  0.40;
             -1,  0,  0,  0.45;
              0,  0, -1,  0.22;
              0,  0,  0,     1];
    g_st = g_st1; %'g_st1' or 'g_st2'
    q_all = real(urInvKin(g_st,type));
    q_origin = ur.get_current_joints();
    [q_best,idx] = findOptimalSolution(q_all,q_origin,type);
    ur.move_joints(q_best,30);
    pause(30);
    disp('Robot reached start pose.');
    q_start = ur.get_current_joints();
else
    % disp('Please manually move the pen tip to the start position of the star...');
    % pause;
    q_start = ur.get_current_joints();
end

q_current = q_start;
g_start_tool0 = urFwdKin(q_start, type);

%% --- 3. Define Star Vertices ---

% 5 outer points of the star
theta = deg2rad([90 162 234 306 18]);

x = R * cos(theta);
y = R * sin(theta);

points = [x; y];

% Connection order for a five-pointed star
order = [1 3 5 2 4 1];

%% --- 4. Begin Drawing Star ---

disp('Starting star drawing...');

for k = 1:length(order)-1
    % Start point
    p1 = points(:, order(k));

    % End point
    p2 = points(:, order(k+1));

    % Translation direction
    delta = p2 - p1;

    % Unit direction
    direction = delta / norm(delta);

    % Speed magnitude
    distance = norm(delta);
    v_mag = distance / T_side;
    vx = direction(1) * v_mag;
    vy = direction(2) * v_mag;
    vz = 0;
    t = 0:dt:T_side;
    disp(['Drawing side number', num2str(k)]);

    for i = 1:length(t)-1
        %% Desired Cartesian velocity
        wx = 0;
        wy = 0;
        wz = 0;
        V_desired = [vx;
                     vy;
                     vz;
                     wx;
                     wy;
                     wz];

        %% Current pen tip position
        g_current_tool0 = urFwdKin(q_current, type);

        g_current_pen = g_current_tool0 * g_t_p;

        p_pen = g_current_pen(1:3,4);

        %% Calculate Pen Tip Jacobian
        J_pen = calculatePenJacobian(q_current, p_pen, type);

        %% Solve for joint velocities
        dq = pinv(J_pen) * V_desired;

        %% Safety velocity limit
        limit = 0.23;
        if max(abs(dq)) > limit
            dq = dq * (limit / max(abs(dq)));
        end

        %% Euler integration
        q_next = q_current + dq * dt;

        %% Execute movement
        ur.move_joints(q_next, dt*1.5);
        q_current = q_next;
        pause(dt*1.5);

    end
end

%% --- 5. Lift Pen ---
disp('Star completed, lifting pen...');
q_current = ur.get_current_joints();
g_lift = urFwdKin(q_current, type);
g_lift(3,4) = g_lift(3,4) + 0.02; % 2cm
sol_lift = urInvKin(g_lift, type);
[~, idx] = min(vecnorm(sol_lift - q_current, 2, 1));
ur.move_joints(sol_lift(:,idx),10.0);
disp('Done.');
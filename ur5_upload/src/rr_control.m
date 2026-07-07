function rr_control(type,operating_mode)
%% Circular trajectory using Resolved-rate Control

%% Initialize robot interface
ur = ur_interface();
pause(0.5);
%% --- 1. Parameters ---
R = 0.05; % ..m
T_total = 20;
dt = 0.2;
t = 0:dt:T_total;

%% Tool offset (pen tip)
d_z = 0.12228;
d_y = -0.049;
g_t_p = [1 0 0   0;
         0 1 0 d_y;
         0 0 1 d_z;
         0 0 0   1];
%% --- 2. Get initial pose ---
if strcmp(operating_mode,'RVIZ')
    g_st1 = [ 0, -1,  0,  0.25;
             -1,  0,  0,  0.60;
              0,  0, -1,  0.22;
              0,  0,  0,     1];
    g_st2 = [ 0, -1,  0,  0.40;
             -1,  0,  0,  0.45;
              0,  0, -1,  0.22;
              0,  0,  0,     1];
    g_st = g_st2; %'g_st1' or 'g_st2'
    q_all = real(urInvKin(g_st,type));
    q_origin = ur.get_current_joints();
    [q_best,idx] = findOptimalSolution(q_all,q_origin,type);
    ur.move_joints(q_best,30);
    pause(30);
    disp('Robot reached start pose.');
    q_start = ur.get_current_joints();
    g_start_tool0 = urFwdKin(q_start, type);
    % --- Start Point Error Analysis in Rviz (Error Reporting) ---
    g_desired = g_st; % Desired start pose
    g_actual = urFwdKin(q_start, type); % Actual start pose
    % Extract desired and actual rotation matrices and position vectors
    R_desired = g_desired(1:3, 1:3); 
    r_desired = g_desired(1:3, 4);
    R_actual = g_actual(1:3, 1:3); 
    r_actual = g_actual(1:3, 4);
    % Calculate Rotation Error (dSO3) and Translation Error (dR3)
    dSO3 = sqrt(trace((R_actual - R_desired) * (R_actual - R_desired)')); % Rotation error in radians
    dR3 = norm(r_actual - r_desired); % Translation error in meters
    % Print analysis results to the console
    fprintf('\n--- IK Start Position Error Analysis ---\ndSO(3): %.6f rad\ndR^3: %.6f meters\n--------------------------------------\n', dSO3, dR3);
else
    % REAL ROBOT MODE
    % disp('Place pen on starting point and press Enter...');
    % pause;
    q_start = ur.get_current_joints();
    g_start_tool0 = urFwdKin(q_start, type);
end
%% Initial state
q_current = q_start;
g_start_pen = g_start_tool0 * g_t_p;
traj_errors = zeros(1, length(t)-1); 

%% --- 3. RR Control Loop ---
disp('Starting trajectory tracking...');
for i = 1:length(t)-1
    % desired circular motion
    omega = 2*pi / T_total;
    angle = omega * t(i);
    
    % --- Define the desired target position (for error calculation) ---
    % According to the rr_control logic, the desired position offset corresponding to the velocity integral
    dx_target = R * (cos(angle) - 1); 
    dy_target = R * sin(angle);
    p_desired = g_start_pen(1:3, 4) + [dx_target; dy_target; 0];
    
    vx = -R * sin(angle) * omega;
    vy =  R * cos(angle) * omega;
    vz = 0;
    wx = 0; wy = 0; wz = 0;
    V_desired = [vx; vy; vz; wx; wy; wz];
    
    % forward kinematics
    g_current_tool0 = urFwdKin(q_current, type);
    g_current_pen = g_current_tool0 * g_t_p;
    p_pen = g_current_pen(1:3,4);
    
    % --- Real-time Trajectory Error Analysis ---
    traj_errors(i) = norm(p_pen - p_desired);
    
    % Jacobian at pen tip
    J_pen = calculatePenJacobian(q_current, p_pen, type);
    % joint velocity
    dq = pinv(J_pen) * V_desired;
    % safety limit
    limit = 0.23;
    if max(abs(dq)) > limit
        dq = dq * (limit / max(abs(dq)));
    end
    % integrate
    q_next = q_current + dq * dt;
    % send command
    ur.move_joints(q_next, dt*1.5);
    q_current = q_next;
    pause(dt);
end

% --- Error Analysis ---
end_pose_error = traj_errors(end);
max_traj_error = max(traj_errors);
avg_traj_error = mean(traj_errors);

fprintf('\n--- Final RR Trajectory Error Summary ---\n');
fprintf('End Pose Error (dR^3): %.2e meters\n', end_pose_error);
fprintf('Max Trajectory Error: %.2e meters\n', max_traj_error);
fprintf('Average Trajectory Error: %.2e meters\n', avg_traj_error);
fprintf('-----------------------------------------\n');

%% --- 4. Lift pen ---
disp('Finished. Lifting pen...');
q_current = ur.get_current_joints();
g_lift = urFwdKin(q_current, type);
g_lift(3,4) = g_lift(3,4) + 0.02;
sol_lift = urInvKin(g_lift, type);
[~, idx_lift] = min(vecnorm(sol_lift - q_current, 2, 1));
ur.move_joints(sol_lift(:,idx_lift), 10.0);
disp('Done.');
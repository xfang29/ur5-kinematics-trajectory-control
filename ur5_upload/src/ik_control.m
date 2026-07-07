function ik_control(type,operating_mode)
%% Circular trajectory using Inverse Kinematics Control

% Initialize robot interface
ur = ur_interface();

%% --- 1. Parameter Settings ---
R = 0.05;         % Radius 
T_total = 50;     % Total duration
dt = 0.1;         % Time step
t = 0:dt:T_total;

% Pen tip offset relative to tool0
d_z = 0.12228;
d_y = -0.049; 
g_t_p = [1 0 0   0; 
         0 1 0 d_y; 
         0 0 1 d_z; 
         0 0 0   1];

%% --- 2. Acquire Initial Pose (Teaching) ---
if strcmp(operating_mode,'RVIZ')
    g_st1 = [ 0, -1,  0,  0.25;
             -1,  0,  0,  0.60;
              0,  0, -1,  0.22;
              0,  0,  0,     1];
    g_st2 = [ 0, -1,  0,  0.40;
              -1,  0,  0,  0.45;
               0,  0, -1,  0.22;
               0,  0,  0,     1];
    g_st = g_st2;
    q_all = real(urInvKin(g_st,type));
    q_origin = ur.get_current_joints();
    [q_best,idx] = findOptimalSolution(q_all,q_origin,type);
    ur.move_joints(q_best,30);
    pause(30);
    disp('Robot has reached the starting point, reading pose...');
    q_start = ur.get_current_joints(); 
    g_start_tool0 = urFwdKin(q_start, type);
    g_start_pen = g_start_tool0 * g_t_p;
    
    % --- Start Point Error Analysis (Deliverable Requirement) ---
    g_desired = g_st;
    g_actual = urFwdKin(q_start, type);
    R_desired = g_desired(1:3, 1:3); 
    r_desired = g_desired(1:3, 4);
    R_actual = g_actual(1:3, 1:3); 
    r_actual = g_actual(1:3, 4);
    dSO3_start = sqrt(trace((R_actual - R_desired) * (R_actual - R_desired)')); 
    dR3_start = norm(r_actual - r_desired); 
    fprintf('\n--- IK Start Position Error Analysis ---\ndSO(3): %.6f rad\ndR^3: %.6f meters\n--------------------------\n', dSO3_start, dR3_start);
else 
    q_start = ur.get_current_joints();  
    g_start_tool0 = urFwdKin(q_start, type);
    g_start_pen = g_start_tool0 * g_t_p;
end

% Trajectory start point
q_current = q_start; 
q_path = zeros(6, length(t));
traj_errors = zeros(1, length(t)); 

%% --- 3. Loop Control ---
for i = 1:length(t)
    % Plan the trajectory in the pen tip coordinate system
    angle = 2 * pi * (t(i) / T_total);
    dx = R * (cos(angle) - 1); 
    dy = R * sin(angle);
    g_target_pen = g_start_pen;
    g_target_pen(1,4) = g_start_pen(1,4) + dx;
    g_target_pen(2,4) = g_start_pen(2,4) + dy;

    % Convert back to tool0
    g_target_tool0 = g_target_pen / g_t_p;
    all_solutions = urInvKin(g_target_tool0, type);
    [q_sol, ~] = findOptimalSolution(all_solutions, q_current, type);
    
    % --- Real-time Trajectory Error Analysis ---
    g_actual_tool0_step = urFwdKin(q_sol, type);
    g_actual_pen_step = g_actual_tool0_step * g_t_p;
    
    p_desired = g_target_pen(1:3, 4);
    p_actual = g_actual_pen_step(1:3, 4);
    traj_errors(i) = norm(p_actual - p_desired);

    % Update status and send commands
    q_path(:,i) = q_sol;
    q_current = q_sol; 
    ur.move_joints(q_sol,dt*1.5);
    pause(dt); 
end

% --- Error Analysis ---
end_pose_error = traj_errors(end);
max_traj_error = max(traj_errors);
avg_traj_error = mean(traj_errors);

fprintf('\n--- Final Trajectory Error Summary ---\n');
fprintf('End Pose Error (dR^3): %.2e meters\n', end_pose_error);
fprintf('Max Trajectory Error: %.2e meters\n', max_traj_error);
fprintf('Average Trajectory Error: %.2e meters\n', avg_traj_error);
fprintf('--------------------------------------\n');

% Lift the pen after drawing (Safety operation)
disp('Finished, lifting pen...');
g_lift = urFwdKin(ur.get_current_joints(), type);
g_lift(3,4) = g_lift(3,4) + 0.05; 
sol_lift = urInvKin(g_lift, type);
[~, idx_lift] = min(vecnorm(sol_lift - q_current, 2, 1));
ur.move_joints(sol_lift(:,idx_lift),2.0);
disp('Done.');
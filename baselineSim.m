% =========================================================================
% BASELINE SIMULATION - MANUALLY TUNED CONTROLLER
% Integrated simulation to reproduce Figure 6 from the paper
% =========================================================================

clear; clc; close all;

% Set default interpreters to LaTeX for professional figures
set(groot, 'defaultTextInterpreter', 'latex');
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
set(groot, 'defaultColorbarTickLabelInterpreter', 'latex');

fprintf('\n===============================================\n');
fprintf('  BASELINE SIMULATION - MANUALLY TUNED PD\n');
fprintf('===============================================\n\n');

%% Initialize Components

% Create quadrotor dynamics model
fprintf('Initializing quadrotor dynamics...\n');
quad = QuadrotorDynamics();

% Create PD controller with manual gains (Table 2)
fprintf('Initializing PD controller with manual gains...\n');
controller = PDController();

% Generate reference trajectory
fprintf('Generating circular reference trajectory...\n');
traj = TrajectoryGenerator();

%% Simulation Parameters
dt = 0.05;              % Sampling time [s]
t_total = 45;           % Total simulation time [s]
N = round(t_total / dt);

% Storage arrays
time = zeros(N, 1);
states = zeros(N, 12);
references = zeros(N, 4);
controls = zeros(N, 4);
errors_pos = zeros(N, 3);
errors_att = zeros(N, 3);

fprintf('\nSimulation parameters:\n');
fprintf('  Duration: %.1f s\n', t_total);
fprintf('  Sampling time: %.3f s\n', dt);
fprintf('  Number of steps: %d\n\n', N);

%% Initial Conditions
% Start on ground at origin
initial_state = zeros(12, 1);
quad.setState(initial_state);

fprintf('Starting simulation...\n');
tic;

%% Main Simulation Loop
for i = 1:N
    % Current time
    t = (i-1) * dt;
    time(i) = t;
    
    % Get current state
    state = quad.getState();
    states(i, :) = state';
    
    % Get reference
    ref = traj.getReference(t);
    references(i, :) = ref';
    
    % Compute control
    control = controller.computeControl(state, ref);
    controls(i, :) = control';
    
    % Compute errors
    errors_pos(i, :) = ref(1:3)' - state(1:3)';
    errors_att(i, :) = [ref(4) - state(6), 0 - state(4), 0 - state(5)];
    
    % Note: For roll and pitch, reference comes from outer loop
    % Here we approximate as zero for error calculation
    
    % Step dynamics
    quad.step(control);
    
    % Progress indicator
    if mod(i, 100) == 0
        fprintf('  Progress: %.1f%%\r', 100*i/N);
    end
end

sim_time = toc;
fprintf('\nSimulation completed in %.2f seconds\n', sim_time);

%% Compute Performance Metrics

% Attitude error norm
attitude_error_norm = vecnorm(errors_att, 2, 2);

% RMSE
rmse_att = sqrt(mean(attitude_error_norm.^2));
rmse_pos_x = sqrt(mean(errors_pos(:,1).^2));
rmse_pos_y = sqrt(mean(errors_pos(:,2).^2));
rmse_pos_z = sqrt(mean(errors_pos(:,3).^2));

fprintf('\n--- Performance Metrics ---\n');
fprintf('Attitude Error Norm RMSE: %.4e rad\n', rmse_att);
fprintf('Position RMSE [x, y, z]: [%.4f, %.4f, %.4f] m\n', ...
        rmse_pos_x, rmse_pos_y, rmse_pos_z);
fprintf('Max attitude error: %.4f rad (%.2f deg)\n', ...
        max(attitude_error_norm), rad2deg(max(attitude_error_norm)));

%% Generate Figures

% Figure 1: Attitude Error Norm (Reproduce Figure 6)
figure('Name', 'Baseline - Attitude Error Norm', 'Position', [100, 100, 900, 400]);
plot(time, attitude_error_norm, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 12);
ylabel('$\|e_\eta\|$ [rad]', 'Interpreter', 'latex', 'FontSize', 12);
title('Attitude Error Norm - Manually Tuned Controller', ...
      'Interpreter', 'latex', 'FontSize', 13);

% Add vertical lines for trajectory phases
t1 = 10; t2 = 12.5; t3 = 32.5; t4 = 35;
hold on;
xline(t1, 'k--', 'LineWidth', 0.8, 'Alpha', 0.3);
xline(t2, 'k--', 'LineWidth', 0.8, 'Alpha', 0.3);
xline(t3, 'k--', 'LineWidth', 0.8, 'Alpha', 0.3);
xline(t4, 'k--', 'LineWidth', 0.8, 'Alpha', 0.3);

% Add text annotations
text(t1, max(attitude_error_norm)*0.9, 'Takeoff End', ...
     'Interpreter', 'latex', 'FontSize', 9, 'HorizontalAlignment', 'center');
text((t2+t3)/2, max(attitude_error_norm)*0.9, 'Circle', ...
     'Interpreter', 'latex', 'FontSize', 9, 'HorizontalAlignment', 'center');

xlim([0, t_total]);
ylim([0, max(attitude_error_norm)*1.05]);

% Save figure
saveas(gcf, 'baseline_attitude_error.png');
fprintf('\nFigure saved: baseline_attitude_error.png\n');

% Figure 2: 3D Trajectory Tracking
figure('Name', 'Baseline - 3D Trajectory', 'Position', [150, 150, 800, 600]);
plot3(references(:,1), references(:,2), references(:,3), ...
      'b--', 'LineWidth', 2, 'DisplayName', 'Reference');
hold on;
plot3(states(:,1), states(:,2), states(:,3), ...
      'r-', 'LineWidth', 1.5, 'DisplayName', 'Actual');
grid on;
xlabel('$x$ [m]', 'Interpreter', 'latex', 'FontSize', 12);
ylabel('$y$ [m]', 'Interpreter', 'latex', 'FontSize', 12);
zlabel('$z$ [m]', 'Interpreter', 'latex', 'FontSize', 12);
title('3D Trajectory Tracking', 'Interpreter', 'latex', 'FontSize', 13);
legend('Interpreter', 'latex', 'Location', 'best', 'FontSize', 10);
axis equal;
view(45, 30);
saveas(gcf, 'baseline_3d_trajectory.png');

% Figure 3: Position Errors
figure('Name', 'Baseline - Position Errors', 'Position', [200, 200, 900, 600]);

subplot(3,1,1);
plot(time, errors_pos(:,1), 'r-', 'LineWidth', 1.5);
grid on;
ylabel('$e_x$ [m]', 'Interpreter', 'latex', 'FontSize', 11);
title('Position Errors', 'Interpreter', 'latex', 'FontSize', 12);
xlim([0, t_total]);

subplot(3,1,2);
plot(time, errors_pos(:,2), 'g-', 'LineWidth', 1.5);
grid on;
ylabel('$e_y$ [m]', 'Interpreter', 'latex', 'FontSize', 11);
xlim([0, t_total]);

subplot(3,1,3);
plot(time, errors_pos(:,3), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 11);
ylabel('$e_z$ [m]', 'Interpreter', 'latex', 'FontSize', 11);
xlim([0, t_total]);

saveas(gcf, 'baseline_position_errors.png');

% Figure 4: Attitude Angles
figure('Name', 'Baseline - Attitude Angles', 'Position', [250, 250, 900, 600]);

subplot(3,1,1);
plot(time, rad2deg(states(:,4)), 'r-', 'LineWidth', 1.5);
grid on;
ylabel('$\phi$ [deg]', 'Interpreter', 'latex', 'FontSize', 11);
title('Attitude Angles', 'Interpreter', 'latex', 'FontSize', 12);
xlim([0, t_total]);

subplot(3,1,2);
plot(time, rad2deg(states(:,5)), 'g-', 'LineWidth', 1.5);
grid on;
ylabel('$\theta$ [deg]', 'Interpreter', 'latex', 'FontSize', 11);
xlim([0, t_total]);

subplot(3,1,3);
plot(time, rad2deg(states(:,6)), 'b-', 'LineWidth', 1.5);
hold on;
plot(time, rad2deg(references(:,4)), 'b--', 'LineWidth', 1.2);
grid on;
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 11);
ylabel('$\psi$ [deg]', 'Interpreter', 'latex', 'FontSize', 11);
legend('Actual', 'Reference', 'Interpreter', 'latex', 'Location', 'best');
xlim([0, t_total]);

saveas(gcf, 'baseline_attitude_angles.png');

% Figure 5: Control Inputs
figure('Name', 'Baseline - Control Inputs', 'Position', [300, 300, 900, 600]);

subplot(4,1,1);
plot(time, controls(:,1), 'k-', 'LineWidth', 1.5);
grid on;
ylabel('$\tau_T$', 'Interpreter', 'latex', 'FontSize', 11);
title('Control Inputs', 'Interpreter', 'latex', 'FontSize', 12);
xlim([0, t_total]);

subplot(4,1,2);
plot(time, controls(:,2), 'r-', 'LineWidth', 1.5);
grid on;
ylabel('$\tau_R$', 'Interpreter', 'latex', 'FontSize', 11);
xlim([0, t_total]);

subplot(4,1,3);
plot(time, controls(:,3), 'g-', 'LineWidth', 1.5);
grid on;
ylabel('$\tau_P$', 'Interpreter', 'latex', 'FontSize', 11);
xlim([0, t_total]);

subplot(4,1,4);
plot(time, controls(:,4), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 11);
ylabel('$\tau_Y$', 'Interpreter', 'latex', 'FontSize', 11);
xlim([0, t_total]);

saveas(gcf, 'baseline_control_inputs.png');

% %% Save Results
% fprintf('\nSaving simulation results...\n');
% results.time = time;
% results.states = states;
% results.references = references;
% results.controls = controls;
% results.errors_pos = errors_pos;
% results.errors_att = errors_att;
% results.attitude_error_norm = attitude_error_norm;
% results.metrics.rmse_att = rmse_att;
% results.metrics.rmse_pos = [rmse_pos_x, rmse_pos_y, rmse_pos_z];
% results.metrics.max_att_error = max(attitude_error_norm);
% 
% save('baseline_results.mat', 'results');
% fprintf('Results saved to: baseline_results.mat\n');
% 
% fprintf('\n===============================================\n');
% fprintf('  BASELINE SIMULATION COMPLETED\n');
% fprintf('===============================================\n');
% fprintf('\nExpected RMSE (from paper Table 4): 12.75e-3 rad\n');
% fprintf('Achieved RMSE: %.4e rad\n', rmse_att);
% if abs(rmse_att - 12.75e-3) / 12.75e-3 < 0.2
%     fprintf('✓ Result within 20%% of paper baseline!\n');
% else
%     fprintf('Note: Difference from paper may be due to implementation details\n');
% end
% 
% fprintf('\nNext steps:\n');
% fprintf('1. Verify baseline performance matches paper Figure 6\n');
% fprintf('2. Implement DDPG RL agent for gain tuning\n');
% fprintf('3. Train agent and compare with baseline\n\n');
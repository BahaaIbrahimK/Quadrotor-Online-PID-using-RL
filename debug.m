% =========================================================================
% DEBUG SCRIPT - Find Source of Instability
% =========================================================================

clear; clc; close all;

fprintf('\n=== DEBUGGING QUADROTOR SIMULATION ===\n\n');

%% Test 1: Quadrotor Dynamics with Zero Control
fprintf('Test 1: Quadrotor free fall (no control)...\n');
quad = QuadrotorDynamics();

% Start at 1m altitude, at rest
initial_state = [0; 0; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0];
quad.setState(initial_state);

% No control (free fall)
for i = 1:20
    state = quad.getState();
    control = [0; 0; 0; 0];  % Zero thrust
    quad.step(control);
    
    if any(isnan(state)) || any(isinf(state))
        fprintf('  ❌ NaN detected at step %d\n', i);
        disp(state);
        return;
    end
end
fprintf('  ✓ Dynamics stable under free fall\n');
fprintf('  Final z position: %.3f m (should be < 1m)\n', quad.state(3));

%% Test 2: Hover Control
fprintf('\nTest 2: Hovering at 1m with correct hover thrust...\n');
quad.resetState();
quad.setState([0; 0; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0]);

% Hover command: tau_T = 1 + m*g/(4*Tmax) for balanced thrust
m = quad.m;
g = quad.g;
Tmax = quad.Tmax;
tau_T_hover = 1 + (m * g) / (4 * Tmax);
fprintf('  Computed hover thrust: tau_T = %.4f\n', tau_T_hover);

for i = 1:100
    control = [tau_T_hover; 0; 0; 0];
    quad.step(control);
    
    if any(isnan(quad.state)) || any(isinf(quad.state))
        fprintf('  ❌ NaN detected at step %d\n', i);
        return;
    end
end
fprintf('  ✓ Hover stable\n');
fprintf('  Final z position: %.3f m (should be ≈ 1m)\n', quad.state(3));
fprintf('  Altitude error: %.3f m\n', abs(quad.state(3) - 1));

%% Test 3: Controller Output at Hover
fprintf('\nTest 3: Controller output for hover at z=1m...\n');
controller = PDController();

state = [0; 0; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0];
reference = [0; 0; 1; 0];  % Hover at origin, 1m altitude

control = controller.computeControl(state, reference);
fprintf('  Control output: [τ_T, τ_R, τ_P, τ_Y] = [%.3f, %.3f, %.3f, %.3f]\n', ...
        control(1), control(2), control(3), control(4));

if any(isnan(control)) || any(isinf(control))
    fprintf('  ❌ NaN in control output!\n');
    return;
end

if abs(control(1) - 1) > 0.5
    fprintf('  ⚠ Warning: Thrust far from hover value (1.0)\n');
end

fprintf('  ✓ Controller produces valid output\n');

%% Test 4: Closed-loop hover
fprintf('\nTest 4: Closed-loop hover for 5 seconds...\n');
quad.resetState();
quad.setState([0; 0; 0.5; 0; 0; 0; 0; 0; 0; 0; 0; 0]);  % Start at 0.5m
controller.resetController();

dt = 0.05;
N = round(5 / dt);
states = zeros(N, 12);
controls = zeros(N, 4);

% Debug first few iterations
fprintf('  Debugging first 5 iterations:\n');
for i = 1:min(5, N)
    state = quad.getState()
    states(i, :) = state';
    
    % Hover reference at 1m
    ref = [0; 0; 1; 0];
    
    % Compute control
    control = controller.computeControl(state, ref)
    controls(i, :) = control';
    
    fprintf('    Iter %d: z=%.3f, vz=%.3f, tau_T=%.4f, error_z=%.3f\n', ...
            i, state(3), state(9), control(1), ref(3)-state(3));
    
    % Step simulation
    quad.step(control);
end

% Continue rest of simulation
for i = 6:N
    state = quad.getState();
    states(i, :) = state';
    
    % Hover reference at 1m
    ref = [0; 0; 1; 0];
    
    % Compute control
    control = controller.computeControl(state, ref);
    controls(i, :) = control';
    
    % Check for issues
    if any(isnan(state)) || any(isinf(state))
        fprintf('  ❌ NaN in state at step %d\n', i);
        fprintf('  Last valid state:\n');
        disp(states(max(1,i-1), :));
        fprintf('  Last control:\n');
        disp(controls(max(1,i-1), :));
        
        % Plot what happened
        figure('Name', 'Failure Analysis');
        subplot(3,1,1);
        plot((0:i-1)*dt, states(1:i, 3), 'b-');
        ylabel('z [m]');
        title('Altitude before failure');
        grid on;
        
        subplot(3,1,2);
        plot((0:i-1)*dt, rad2deg(states(1:i, 4:6)));
        ylabel('Angles [deg]');
        legend('\phi', '\theta', '\psi');
        grid on;
        
        subplot(3,1,3);
        plot((0:i-1)*dt, controls(1:i, :));
        ylabel('Control');
        xlabel('Time [s]');
        legend('\tau_T', '\tau_R', '\tau_P', '\tau_Y');
        grid on;
        return;
    end
end

fprintf('  ✓ Closed-loop hover stable!\n');
fprintf('  Final altitude: %.3f m (target: 1m)\n', states(end, 3));
fprintf('  Final angles: φ=%.2f°, θ=%.2f°, ψ=%.2f°\n', ...
        rad2deg(states(end, 4)), rad2deg(states(end, 5)), rad2deg(states(end, 6)));
fprintf('  Average tau_T: %.4f (hover should be ~1.349)\n', mean(controls(:,1)));

% Plot results
figure('Name', 'Hover Test Results');
subplot(2,1,1);
plot((0:N-1)*dt, states(:,3), 'b-', 'LineWidth', 1.5);
hold on;
yline(1, 'r--', 'LineWidth', 1);
ylabel('Altitude [m]');
title('Hover Test - Altitude Tracking');
legend('Actual', 'Reference');
grid on;

subplot(2,1,2);
plot((0:N-1)*dt, rad2deg(states(:,4:6)), 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Angles [deg]');
legend('\phi', '\theta', '\psi');
title('Attitude Angles');
grid on;

%% Test 5: Short trajectory following
fprintf('\nTest 5: Following take-off trajectory (10 seconds)...\n');
quad.resetState();
quad.setState([0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0]);
controller.resetController();

traj = TrajectoryGenerator();
dt = 0.05;
t_test = 10;  % First 10 seconds (take-off only)
N = round(t_test / dt);

states = zeros(N, 12);
controls = zeros(N, 4);
errors = zeros(N, 1);

for i = 1:N
    t = (i-1) * dt;
    state = quad.getState();
    states(i, :) = state';
    
    % Get reference
    ref = traj.getReference(t);
    
    % Compute control
    control = controller.computeControl(state, ref);
    controls(i, :) = control';
    
    % Compute error
    e_pos = ref(1:3) - state(1:3);
    e_att = [ref(4) - state(6), 0 - state(4), 0 - state(5)];
    errors(i) = norm(e_att);
    
    % Check for issues
    if any(isnan(state)) || any(isinf(state)) || norm(state) > 1e6
        fprintf('  ❌ Instability at t=%.2fs (step %d)\n', t, i);
        fprintf('  State norm: %.2e\n', norm(state));
        fprintf('  Control: [%.3f, %.3f, %.3f, %.3f]\n', control(1), control(2), control(3), control(4));
        
        % Find when it started
        valid_idx = find(~isnan(errors) & errors < 1e3, 1, 'last');
        if ~isempty(valid_idx)
            fprintf('  Last valid state at step %d (t=%.2fs)\n', valid_idx, valid_idx*dt);
        end
        return;
    end
    
    % Step simulation
    quad.step(control);
end

fprintf('  ✓ Take-off trajectory successful!\n');
fprintf('  Final altitude: %.3f m\n', states(end, 3));
fprintf('  Max attitude error: %.4f rad (%.2f°)\n', max(errors), rad2deg(max(errors)));

% Plot
figure('Name', 'Take-off Test');
subplot(2,1,1);
plot((0:N-1)*dt, states(:,3), 'b-', 'LineWidth', 1.5);
ylabel('Altitude [m]');
title('Take-off Test');
grid on;

subplot(2,1,2);
plot((0:N-1)*dt, errors, 'r-', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Attitude Error Norm [rad]');
grid on;

fprintf('\n=== ALL TESTS PASSED ===\n');
fprintf('The system is stable. You can now run baseline_simulation.m\n\n');
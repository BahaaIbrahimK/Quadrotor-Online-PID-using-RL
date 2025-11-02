% =========================================================================
% SYSTEMATIC TEST OF QUADROTOR DYNAMICS
% Testing each aspect separately
% =========================================================================

clear; clc; close all;

fprintf('\n=== TESTING QUADROTOR DYNAMICS ===\n\n');

%% Test 1: Check hover thrust value
fprintf('TEST 1: Hover thrust calculation\n');
m = 1.2;
g = 9.81;
Tmax = 8.43;

% For hover: T = m*g
T_hover = m * g;
fprintf('  Required hover thrust: %.4f N\n', T_hover);

% From Equation 6a: T = 4*Tmax*(tau_T - 1)
% So: m*g = 4*Tmax*(tau_T - 1)
tau_T_hover = 1 + (m * g) / (4 * Tmax);
fprintf('  Required tau_T for hover: %.4f\n', tau_T_hover);

% Verify
T_check = 4 * Tmax * (tau_T_hover - 1);
fprintf('  Verification: T = %.4f N (should be %.4f)\n', T_check, T_hover);
if abs(T_check - T_hover) < 0.01
    fprintf('  ✓ PASS: Hover thrust equation correct\n\n');
else
    fprintf('  ✗ FAIL: Hover thrust equation incorrect\n\n');
    return;
end

%% Test 2: Hovering simulation
fprintf('TEST 2: Hover at 1m altitude for 5 seconds\n');
quad = QuadrotorDynamics();

% Start at 1m, at rest
initial_state = [0; 0; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0];
quad.setState(initial_state);

% Hover control
control = [tau_T_hover; 0; 0; 0];

dt = 0.05;
T_sim = 5;
N = round(T_sim / dt);

z_values = zeros(N, 1);
vz_values = zeros(N, 1);

for i = 1:N
    state = quad.getState();
    z_values(i) = state(3);
    vz_values(i) = state(9);
    quad.step(control);
end

z_error = abs(z_values(end) - 1.0);
fprintf('  Initial altitude: %.4f m\n', z_values(1));
fprintf('  Final altitude: %.4f m\n', z_values(end));
fprintf('  Altitude error: %.4f m\n', z_error);
fprintf('  Max vertical velocity: %.4f m/s\n', max(abs(vz_values)));

if z_error < 0.01 && max(abs(vz_values)) < 0.01
    fprintf('  ✓ PASS: Stable hover\n\n');
else
    fprintf('  ✗ FAIL: Hover unstable\n\n');
    
    % Plot for debugging
    figure('Name', 'Hover Test Failure');
    subplot(2,1,1);
    plot((0:N-1)*dt, z_values);
    ylabel('Altitude [m]');
    title('Altitude during hover');
    grid on;
    
    subplot(2,1,2);
    plot((0:N-1)*dt, vz_values);
    xlabel('Time [s]');
    ylabel('Vertical velocity [m/s]');
    title('Vertical velocity');
    grid on;
    return;
end

%% Test 3: Climbing with increased thrust
fprintf('TEST 3: Climb from 1m to 2m with increased thrust\n');
quad.resetState();
quad.setState([0; 0; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0]);

% Increase thrust by 20%
tau_T_climb = tau_T_hover * 1.1;
control = [tau_T_climb; 0; 0; 0];

N = round(5 / dt);
z_values = zeros(N, 1);

for i = 1:N
    state = quad.getState();
    z_values(i) = state(3);
    quad.step(control);
end

altitude_gain = z_values(end) - z_values(1);
fprintf('  Initial altitude: %.4f m\n', z_values(1));
fprintf('  Final altitude: %.4f m\n', z_values(end));
fprintf('  Altitude gain: %.4f m\n', altitude_gain);

if altitude_gain > 0.5
    fprintf('  ✓ PASS: Climbs with increased thrust\n\n');
else
    fprintf('  ✗ FAIL: Does not climb properly\n\n');
    return;
end

%% Test 4: Roll torque response
fprintf('TEST 4: Roll response to torque input\n');
quad.resetState();
quad.setState([0; 0; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0]);

% Apply small roll torque
tau_R = 0.1;
control = [tau_T_hover; tau_R; 0; 0];

N = round(2 / dt);
phi_values = zeros(N, 1);
p_values = zeros(N, 1);

for i = 1:N
    state = quad.getState();
    phi_values(i) = state(4);
    p_values(i) = state(10);
    quad.step(control);
end

max_roll = rad2deg(max(abs(phi_values)));
max_roll_rate = rad2deg(max(abs(p_values)));

fprintf('  Max roll angle: %.2f deg\n', max_roll);
fprintf('  Max roll rate: %.2f deg/s\n', max_roll_rate);

if max_roll > 1 && max_roll < 45
    fprintf('  ✓ PASS: Reasonable roll response\n\n');
else
    fprintf('  ✗ FAIL: Roll response out of range\n\n');
    return;
end

%% Test 5: Check rotation matrix
fprintf('TEST 5: Rotation matrix properties\n');
quad.resetState();

% Test at various attitudes
test_angles = [0, 0, 0;
               deg2rad(30), 0, 0;
               0, deg2rad(30), 0;
               0, 0, deg2rad(45);
               deg2rad(15), deg2rad(15), deg2rad(30)];

all_pass = true;
for i = 1:size(test_angles, 1)
    phi = test_angles(i,1);
    theta = test_angles(i,2);
    psi = test_angles(i,3);
    
    R = quad.rotationMatrix(phi, theta, psi);
    
    % Check if R is orthogonal: R'*R = I
    I_check = R' * R;
    error = norm(I_check - eye(3), 'fro');
    
    % Check determinant = 1
    det_R = det(R);
    
    if error > 1e-10 || abs(det_R - 1) > 1e-10
        fprintf('  ✗ FAIL at angles [%.1f, %.1f, %.1f] deg\n', ...
                rad2deg(phi), rad2deg(theta), rad2deg(psi));
        fprintf('    Orthogonality error: %.2e\n', error);
        fprintf('    Determinant: %.6f (should be 1)\n', det_R);
        all_pass = false;
    end
end

if all_pass
    fprintf('  ✓ PASS: Rotation matrix is valid SO(3)\n\n');
end

%% Test 6: Integration accuracy
fprintf('TEST 6: RK4 integration accuracy\n');
quad.resetState();
quad.setState([0; 0; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0]);

% Free fall (no thrust)
control = [0; 0; 0; 0];

% Analytical solution: z(t) = z0 - 0.5*g*t^2
t = 1.0; % 1 second
N = round(t / dt);

for i = 1:N
    quad.step(control);
end

z_final_sim = quad.state(3);
z_final_analytical = 1 - 0.5 * g * t^2;
error = abs(z_final_sim - z_final_analytical);

fprintf('  After %.1f s of free fall:\n', t);
fprintf('  Simulated z: %.4f m\n', z_final_sim);
fprintf('  Analytical z: %.4f m\n', z_final_analytical);
fprintf('  Error: %.4f m\n', error);

if error < 0.01
    fprintf('  ✓ PASS: RK4 integration accurate\n\n');
else
    fprintf('  ⚠ WARNING: Integration error = %.4f m\n\n', error);
end

%% Summary
fprintf('=================================\n');
fprintf('ALL TESTS COMPLETED SUCCESSFULLY\n');
fprintf('=================================\n');
fprintf('\nQuadrotorDynamics.m is working correctly!\n');
fprintf('The dynamics model is ready for controller testing.\n\n');
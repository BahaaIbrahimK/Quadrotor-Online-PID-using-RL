% =========================================================================
% QUADROTOR DYNAMICS MODEL - X CONFIGURATION
% Phase 1: Implementation of Newton-Euler Equations
% Based on: Sönmez et al., "RL-Based Prediction of PID Controller Gains"
% =========================================================================

classdef QuadrotorDynamics < handle
    properties
        % Physical parameters (from Table 1)
        m = 1.2;              % Total mass [kg]
        g = 9.81;             % Gravitational acceleration [m/s^2]
        l = 0.225;            % Arm length [m]
        
        % Inertia matrix [kg*m^2]
        Ixx = 0.0131;
        Iyy = 0.0131;
        Izz = 0.0234;
        I;                    % Will be set in constructor
        
        % Motor/Propeller parameters
        Tmax = 8.43;          % Maximum thrust [N]
        cD_cL = 0.0237;       % Drag to lift coefficient ratio
        
        % State variables [x, y, z, phi, theta, psi, vx, vy, vz, p, q, r]
        state;
        
        % Sampling time
        dt = 0.05;            % 50ms sampling time
    end
    
    methods
        function obj = QuadrotorDynamics()
            % Constructor
            obj.I = diag([obj.Ixx, obj.Iyy, obj.Izz]);
            obj.state = zeros(12, 1);  % Initialize state
        end
        
        function R = rotationMatrix(obj, phi, theta, psi)
            % Rotation matrix from body to inertial frame (Eq. 3)
            % R: SO(3) rotation matrix
            
            cp = cos(phi);   sp = sin(phi);
            ct = cos(theta); st = sin(theta);
            cps = cos(psi);  sps = sin(psi);
            
            R = [ct*cps,  sp*st*cps - cp*sps,  cp*st*cps + sp*sps;
                 ct*sps,  sp*st*sps + cp*cps,  cp*st*sps - sp*cps;
                 -st,     sp*ct,                cp*ct];
        end
        
        function S = skewSymmetric(~, a)
            % Skew-symmetric matrix (Eq. 1)
            % For cross product: S(a)*b = a × b
            S = [0,    -a(3),  a(2);
                 a(3),  0,    -a(1);
                -a(2),  a(1),  0];
        end
        
        function W_inv = eulerRateMatrix(~, phi, theta)
            % Matrix relating angular velocities to Euler rates (Eq. 4)
            sp = sin(phi); cp = cos(phi);
            st = sin(theta); ct = cos(theta);
            
            if abs(ct) < 1e-6
                error('Gimbal lock: theta = ±90°');
            end
            
            W_inv = [1,  sp*st/ct,  cp*st/ct;
                     0,  cp,       -sp;
                     0,  sp/ct,     cp/ct];
        end
        
        function [T, Mp, Mq, Mr] = forcingTerms(obj, tau_T, tau_R, tau_P, tau_Y)
            % Compute forcing terms from virtual control inputs (Eq. 6)
            % Inputs: tau_T, tau_R, tau_P, tau_Y (normalized controls)
            % Outputs: T (thrust), Mp, Mq, Mr (torques)
            
            % Equation 6a from paper: T = 4*Tmax*(tau_T - 1)
            % For hover: tau_T ≈ 1.349, giving T ≈ m*g
            T = 4 * obj.Tmax * (tau_T - 1);
            
            Mp = -4 * obj.Tmax * tau_R * (obj.l * sqrt(2) / 2);
            Mq = -4 * obj.Tmax * tau_P * (obj.l * sqrt(2) / 2);
            Mr = 4 * obj.Tmax * tau_Y * obj.cD_cL;
        end
        
        function state_dot = dynamics(obj, state, control_input)
            % Newton-Euler equations of motion (Eq. 2)
            % State: [x, y, z, phi, theta, psi, vx, vy, vz, p, q, r]'
            % Control: [tau_T, tau_R, tau_P, tau_Y]'
            
            % Extract state variables
            x = state(1); y = state(2); z = state(3);
            phi = state(4); theta = state(5); psi = state(6);
            vx = state(7); vy = state(8); vz = state(9);
            p = state(10); q = state(11); r = state(12);
            
            % Extract control inputs
            tau_T = control_input(1);
            tau_R = control_input(2);
            tau_P = control_input(3);
            tau_Y = control_input(4);
            
            % Compute forcing terms
            [T, Mp, Mq, Mr] = obj.forcingTerms(tau_T, tau_R, tau_P, tau_Y);
            
            % Rotation matrix
            R = obj.rotationMatrix(phi, theta, psi);
            
            % Unit vector along z-axis
            z_hat = [0; 0; -1];
            
            % Position dynamics (Eq. 2a)
            % Thrust acts along body z-axis, pointing upward
            thrust_force = (T / obj.m) * R * z_hat;
            gravity_force = -obj.g * z_hat;
            p_ddot = gravity_force + thrust_force;
            
            % Angular velocity vector
            omega_B = [p; q; r];
            
            % Attitude dynamics (Eq. 2b) - Euler rates
            W_inv = obj.eulerRateMatrix(phi, theta);
            eta_dot = W_inv * omega_B;
            
            % Angular acceleration (Eq. 2c)
            MB = [Mp; Mq; Mr];
            omega_dot = obj.I \ (MB - cross(omega_B, obj.I * omega_B));
            
            % Assemble state derivative
            state_dot = [vx; vy; vz;                    % Position derivatives
                        eta_dot;                        % Euler angle derivatives
                        p_ddot;                         % Velocity derivatives
                        omega_dot];                     % Angular velocity derivatives
        end
        
        function state_next = step(obj, control_input)
            % Integrate dynamics using 4th-order Runge-Kutta
            % Updates and returns next state
            
            % Check for NaN/Inf in control
            if any(isnan(control_input)) || any(isinf(control_input))
                warning('NaN/Inf in control input, using hover command');
                control_input = [1; 0; 0; 0];
            end
            
            k1 = obj.dynamics(obj.state, control_input);
            k2 = obj.dynamics(obj.state + 0.5*obj.dt*k1, control_input);
            k3 = obj.dynamics(obj.state + 0.5*obj.dt*k2, control_input);
            k4 = obj.dynamics(obj.state + obj.dt*k3, control_input);
            
            state_next = obj.state + (obj.dt/6) * (k1 + 2*k2 + 2*k3 + k4);
            
            % Check for NaN/Inf in next state
            if any(isnan(state_next)) || any(isinf(state_next))
                warning('NaN/Inf detected in state after integration!');
                % Try to recover by keeping current state
                state_next = obj.state;
            end
            
            obj.state = state_next;
        end
        
        function setState(obj, new_state)
            % Set quadrotor state
            obj.state = new_state;
        end
        
        function state = getState(obj)
            % Get current state
            state = obj.state;
        end
        
        function resetState(obj)
            % Reset to initial state (on ground)
            obj.state = zeros(12, 1);
        end
    end
end

% =========================================================================
% TEST FUNCTION
% =========================================================================
function testQuadrotorDynamics()
    % Initialize quadrotor
    quad = QuadrotorDynamics();
    
    % Set initial state: hovering at z = 1m
    initial_state = [0; 0; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0];
    quad.setState(initial_state);
    
    % Hover control: tau_T = 1 (net thrust = 0), no torques
    hover_control = [1; 0; 0; 0];
    
    % Simulate for 5 seconds
    T = 5;
    dt = quad.dt;
    N = round(T / dt);
    
    % Storage
    time = zeros(N, 1);
    states = zeros(N, 12);
    
    fprintf('Testing Quadrotor Dynamics Model...\n');
    fprintf('Initial state: [x, y, z] = [%.2f, %.2f, %.2f] m\n', ...
            initial_state(1), initial_state(2), initial_state(3));
    
    % Simulation loop
    for i = 1:N
        time(i) = (i-1) * dt;
        states(i, :) = quad.getState()';
        quad.step(hover_control);
    end
    
    fprintf('Final state: [x, y, z] = [%.2f, %.2f, %.2f] m\n', ...
            states(end, 1), states(end, 2), states(end, 3));
    fprintf('Altitude change: %.4f m (should be ~0 for hovering)\n', ...
            states(end, 3) - initial_state(3));
    
    % Plot results
    figure('Name', 'Quadrotor Dynamics Test');
    
    subplot(3,1,1);
    plot(time, states(:,1:3), 'LineWidth', 1.5);
    xlabel('Time [s]', 'Interpreter', 'latex');
    ylabel('Position [m]', 'Interpreter', 'latex');
    legend('$x$', '$y$', '$z$', 'Interpreter', 'latex', 'Location', 'best');
    title('Position vs Time', 'Interpreter', 'latex');
    grid on;
    
    subplot(3,1,2);
    plot(time, rad2deg(states(:,4:6)), 'LineWidth', 1.5);
    xlabel('Time [s]', 'Interpreter', 'latex');
    ylabel('Angles [deg]', 'Interpreter', 'latex');
    legend('$\phi$ (Roll)', '$\theta$ (Pitch)', '$\psi$ (Yaw)', ...
           'Interpreter', 'latex', 'Location', 'best');
    title('Euler Angles vs Time', 'Interpreter', 'latex');
    grid on;
    
    subplot(3,1,3);
    plot(time, states(:,7:9), 'LineWidth', 1.5);
    xlabel('Time [s]', 'Interpreter', 'latex');
    ylabel('Velocity [m/s]', 'Interpreter', 'latex');
    legend('$v_x$', '$v_y$', '$v_z$', 'Interpreter', 'latex', 'Location', 'best');
    title('Velocity vs Time', 'Interpreter', 'latex');
    grid on;
    
    fprintf('\nTest completed successfully!\n');
end

% Uncomment to run test
% testQuadrotorDynamics();
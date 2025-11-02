% =========================================================================
% CIRCULAR TRAJECTORY GENERATOR
% Phase 3: Reference trajectory for quadrotor tracking
% Trajectory: Take-off → Hover → Circle → Hover → Landing
% =========================================================================

classdef TrajectoryGenerator < handle
    properties
        % Trajectory parameters
        dt = 0.05;            % Sampling time [s]
        
        % Time segments
        t_takeoff = 10;       % Take-off duration [s]
        t_hover1 = 2.5;       % First hover duration [s]
        t_circle = 20;        % Circle duration [s]
        t_hover2 = 2.5;       % Second hover duration [s]
        t_landing = 10;       % Landing duration [s]
        t_total = 45;         % Total duration [s]
        
        % Altitude parameters
        h_cruise = 3;         % Cruise altitude [m]
        h_ground = 0;         % Ground level [m]
        
        % Circle parameters
        circle_radius = 2;    % Circle radius [m]
        circle_center = [0; 0]; % Circle center [x, y] [m]
        
        % Trajectory data
        time;
        position;             % [x, y, z]
        yaw;                  % psi
        velocity;             % [vx, vy, vz]
        acceleration;         % [ax, ay, az]
    end
    
    methods
        function obj = TrajectoryGenerator()
            % Constructor - Generate trajectory
            obj.generateTrajectory();
        end
        
        function generateTrajectory(obj)
            % Generate complete trajectory
            
            % Time vector
            N = round(obj.t_total / obj.dt);
            obj.time = (0:N-1)' * obj.dt;
            
            % Initialize arrays
            obj.position = zeros(N, 3);
            obj.yaw = zeros(N, 1);
            obj.velocity = zeros(N, 3);
            obj.acceleration = zeros(N, 3);
            
            % Time boundaries
            t1 = obj.t_takeoff;
            t2 = t1 + obj.t_hover1;
            t3 = t2 + obj.t_circle;
            t4 = t3 + obj.t_hover2;
            t5 = t4 + obj.t_landing;
            
            fprintf('Generating circular trajectory...\n');
            fprintf('Segments: Takeoff(%.1fs) → Hover(%.1fs) → Circle(%.1fs) → Hover(%.1fs) → Landing(%.1fs)\n', ...
                    obj.t_takeoff, obj.t_hover1, obj.t_circle, obj.t_hover2, obj.t_landing);
            
            for i = 1:N
                t = obj.time(i);
                
                if t <= t1
                    % ===== TAKE-OFF =====
                    [obj.position(i,:), obj.velocity(i,:), obj.acceleration(i,:)] = ...
                        obj.takeoffTrajectory(t, t1);
                    obj.yaw(i) = 0;
                    
                elseif t <= t2
                    % ===== HOVER 1 =====
                    obj.position(i,:) = [obj.circle_center(1), obj.circle_center(2), obj.h_cruise];
                    obj.velocity(i,:) = [0, 0, 0];
                    obj.acceleration(i,:) = [0, 0, 0];
                    obj.yaw(i) = 0;
                    
                elseif t <= t3
                    % ===== CIRCULAR TRAJECTORY =====
                    t_local = t - t2;
                    [obj.position(i,:), obj.velocity(i,:), obj.acceleration(i,:), obj.yaw(i)] = ...
                        obj.circularTrajectory(t_local, obj.t_circle);
                    
                elseif t <= t4
                    % ===== HOVER 2 =====
                    obj.position(i,:) = [obj.circle_center(1), obj.circle_center(2), obj.h_cruise];
                    obj.velocity(i,:) = [0, 0, 0];
                    obj.acceleration(i,:) = [0, 0, 0];
                    obj.yaw(i) = 2*pi;  % Completed one revolution
                    
                else
                    % ===== LANDING =====
                    t_local = t - t4;
                    [obj.position(i,:), obj.velocity(i,:), obj.acceleration(i,:)] = ...
                        obj.landingTrajectory(t_local, obj.t_landing);
                    obj.yaw(i) = 2*pi;
                end
            end
            
            fprintf('Trajectory generated: %d points at %.3fs sampling\n', N, obj.dt);
        end
        
        function [pos, vel, acc] = takeoffTrajectory(obj, t, duration)
            % Smooth take-off trajectory using polynomial
            % 5th order polynomial for smooth acceleration profile
            
            s = t / duration;  % Normalized time [0, 1]
            s = max(0, min(1, s));
            
            % 5th order polynomial: p(s) = 10s^3 - 15s^4 + 6s^5
            p = 10*s^3 - 15*s^4 + 6*s^5;
            dp = (30*s^2 - 60*s^3 + 30*s^4) / duration;
            ddp = (60*s - 180*s^2 + 120*s^3) / duration^2;
            
            % Position
            z = obj.h_ground + p * (obj.h_cruise - obj.h_ground);
            pos = [obj.circle_center(1), obj.circle_center(2), z];
            
            % Velocity
            vz = dp * (obj.h_cruise - obj.h_ground);
            vel = [0, 0, vz];
            
            % Acceleration
            az = ddp * (obj.h_cruise - obj.h_ground);
            acc = [0, 0, az];
        end
        
        function [pos, vel, acc, yaw] = circularTrajectory(obj, t, duration)
            % Circular trajectory at constant altitude
            % One complete revolution
            
            % Angular position (0 to 2π)
            omega = 2*pi / duration;  % Angular velocity [rad/s]
            theta = omega * t;
            
            % Position
            x = obj.circle_center(1) + obj.circle_radius * cos(theta);
            y = obj.circle_center(2) + obj.circle_radius * sin(theta);
            z = obj.h_cruise;
            pos = [x, y, z];
            
            % Velocity
            vx = -obj.circle_radius * omega * sin(theta);
            vy = obj.circle_radius * omega * cos(theta);
            vz = 0;
            vel = [vx, vy, vz];
            
            % Acceleration (centripetal)
            ax = -obj.circle_radius * omega^2 * cos(theta);
            ay = -obj.circle_radius * omega^2 * sin(theta);
            az = 0;
            acc = [ax, ay, az];
            
            % Yaw angle (tangent to circle)
            yaw = theta + pi/2;  % Point in direction of motion
        end
        
        function [pos, vel, acc] = landingTrajectory(obj, t, duration)
            % Smooth landing trajectory using polynomial
            
            s = t / duration;  % Normalized time [0, 1]
            s = max(0, min(1, s));
            
            % 5th order polynomial
            p = 10*s^3 - 15*s^4 + 6*s^5;
            dp = (30*s^2 - 60*s^3 + 30*s^4) / duration;
            ddp = (60*s - 180*s^2 + 120*s^3) / duration^2;
            
            % Position (descending from cruise altitude to ground)
            z = obj.h_cruise - p * (obj.h_cruise - obj.h_ground);
            pos = [obj.circle_center(1), obj.circle_center(2), z];
            
            % Velocity
            vz = -dp * (obj.h_cruise - obj.h_ground);
            vel = [0, 0, vz];
            
            % Acceleration
            az = -ddp * (obj.h_cruise - obj.h_ground);
            acc = [0, 0, az];
        end
        
        function ref = getReference(obj, t)
            % Get reference at time t
            % Returns: [xr, yr, zr, psir]'
            
            idx = find(obj.time >= t, 1, 'first');
            if isempty(idx)
                idx = length(obj.time);
            end
            
            ref = [obj.position(idx, 1);
                   obj.position(idx, 2);
                   obj.position(idx, 3);
                   obj.yaw(idx)];
        end
        
        function plotTrajectory(obj)
            % Visualize the generated trajectory
            
            figure('Name', 'Reference Trajectory', 'Position', [100, 100, 1200, 800]);
            
            % 3D trajectory
            subplot(2, 2, 1);
            plot3(obj.position(:,1), obj.position(:,2), obj.position(:,3), ...
                  'b-', 'LineWidth', 2);
            hold on;
            plot3(obj.position(1,1), obj.position(1,2), obj.position(1,3), ...
                  'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
            plot3(obj.position(end,1), obj.position(end,2), obj.position(end,3), ...
                  'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
            grid on;
            xlabel('$x$ [m]', 'Interpreter', 'latex');
            ylabel('$y$ [m]', 'Interpreter', 'latex');
            zlabel('$z$ [m]', 'Interpreter', 'latex');
            title('3D Trajectory', 'Interpreter', 'latex');
            legend('Trajectory', 'Start', 'End', 'Interpreter', 'latex', 'Location', 'best');
            axis equal;
            view(45, 30);
            
            % Position vs time
            subplot(2, 2, 2);
            plot(obj.time, obj.position(:,1), 'r-', 'LineWidth', 1.5); hold on;
            plot(obj.time, obj.position(:,2), 'g-', 'LineWidth', 1.5);
            plot(obj.time, obj.position(:,3), 'b-', 'LineWidth', 1.5);
            grid on;
            xlabel('Time [s]', 'Interpreter', 'latex');
            ylabel('Position [m]', 'Interpreter', 'latex');
            title('Position vs Time', 'Interpreter', 'latex');
            legend('$x$', '$y$', '$z$', 'Interpreter', 'latex', 'Location', 'best');
            
            % Velocity vs time
            subplot(2, 2, 3);
            plot(obj.time, obj.velocity(:,1), 'r-', 'LineWidth', 1.5); hold on;
            plot(obj.time, obj.velocity(:,2), 'g-', 'LineWidth', 1.5);
            plot(obj.time, obj.velocity(:,3), 'b-', 'LineWidth', 1.5);
            grid on;
            xlabel('Time [s]', 'Interpreter', 'latex');
            ylabel('Velocity [m/s]', 'Interpreter', 'latex');
            title('Velocity vs Time', 'Interpreter', 'latex');
            legend('$v_x$', '$v_y$', '$v_z$', 'Interpreter', 'latex', 'Location', 'best');
            
            % Yaw angle vs time
            subplot(2, 2, 4);
            plot(obj.time, rad2deg(obj.yaw), 'k-', 'LineWidth', 1.5);
            grid on;
            xlabel('Time [s]', 'Interpreter', 'latex');
            ylabel('Yaw Angle [deg]', 'Interpreter', 'latex');
            title('Yaw Angle vs Time', 'Interpreter', 'latex');
            
            % Add vertical lines for phase transitions
            t1 = obj.t_takeoff;
            t2 = t1 + obj.t_hover1;
            t3 = t2 + obj.t_circle;
            t4 = t3 + obj.t_hover2;
            
            for sp = 2:4
                subplot(2, 2, sp);
                hold on;
                xline(t1, 'k--', 'LineWidth', 0.5);
                xline(t2, 'k--', 'LineWidth', 0.5);
                xline(t3, 'k--', 'LineWidth', 0.5);
                xline(t4, 'k--', 'LineWidth', 0.5);
            end
        end
    end
end

% =========================================================================
% TEST FUNCTION
% =========================================================================
function testTrajectoryGenerator()
    % Test trajectory generation
    
    fprintf('\n=== Testing Trajectory Generator ===\n');
    
    % Create trajectory
    traj = TrajectoryGenerator();
    
    % Display statistics
    fprintf('\nTrajectory Statistics:\n');
    fprintf('  Total duration: %.1f s\n', traj.t_total);
    fprintf('  Total points: %d\n', length(traj.time));
    fprintf('  Max altitude: %.2f m\n', max(traj.position(:,3)));
    fprintf('  Circle radius: %.2f m\n', traj.circle_radius);
    fprintf('  Max velocity: %.2f m/s\n', max(vecnorm(traj.velocity, 2, 2)));
    
    % Plot trajectory
    traj.plotTrajectory();
    
    % Test reference lookup
    test_times = [0, 10, 15, 32.5, 45];
    fprintf('\nReference at specific times:\n');
    for i = 1:length(test_times)
        ref = traj.getReference(test_times(i));
        fprintf('  t = %.1fs: [x, y, z, psi] = [%.2f, %.2f, %.2f, %.2f°]\n', ...
                test_times(i), ref(1), ref(2), ref(3), rad2deg(ref(4)));
    end
    
    fprintf('\nTest completed!\n');
end

% Uncomment to run test
% testTrajectoryGenerator();
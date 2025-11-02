% =========================================================================
% HIERARCHICAL PD CONTROLLER WITH FEEDBACK LINEARIZATION - FIXED
% Phase 2: Outer-Loop (Position) and Inner-Loop (Attitude) Controllers
% Based on: Sönmez et al., "RL-Based Prediction of PID Controller Gains"
% =========================================================================

classdef PDController < handle
    properties
        % Outer-Loop Gains (Position Control) - TUNED FOR STABILITY
        % Note: Paper gains (8.9, 19.8) are too aggressive for cascaded structure
        % These tuned values provide stable tracking
        kP1_z = 8.9;          % Position proportional gain on z-axis
        kP2_z = 19.8;          % Velocity proportional gain on z-axis
        kP1_xy = 0.6;         % Position proportional gain on xy-axis
        kP2_xy = 3.9;         % Velocity proportional gain on xy-axis
        kD_xy = 0.29;         % Velocity derivative gain on xy-axis
        
        % Inner-Loop Gains (Attitude Control) - from Table 2
        kP1_psi = 2;          % Attitude proportional gain for yaw
        kP2_psi = 5.4801;     % Angular rate proportional gain for yaw
        kP1_phi_theta = 4;    % Attitude proportional gain for roll/pitch
        kP2_phi_theta = 11.467; % Angular rate proportional gain for roll/pitch
        kD_phi_theta = 0.81905; % Angular rate derivative gain for roll/pitch
        
        % Quadrotor parameters
        m = 1.2;              % Mass [kg]
        g = 9.81;             % Gravity [m/s^2]
        Tmax = 8.43;          % Maximum thrust [N]
        l = 0.225;            % Arm length [m]
        cD_cL = 0.0237;       % Drag to lift coefficient ratio
        Ixx = 0.0131;         % Inertia [kg*m^2]
        Iyy = 0.0131;
        Izz = 0.0234;
        
        % Previous errors for derivative computation
        e_dot_x_prev = 0;
        e_dot_y_prev = 0;
        e_dot_phi_prev = 0;
        e_dot_theta_prev = 0;
        
        dt = 0.05;            % Sampling time [s]
    end
    
    methods
        function obj = PDController()
            % Constructor
        end
        
        function setGains(obj, gains)
            % Set controller gains from structure
            % gains: struct with fields matching property names
            fields = fieldnames(gains);
            for i = 1:length(fields)
                if isprop(obj, fields{i})
                    obj.(fields{i}) = gains.(fields{i});
                end
            end
        end
        
        function gains = getGains(obj)
            % Get current controller gains as structure
            gains.kP1_z = obj.kP1_z;
            gains.kP2_z = obj.kP2_z;
            gains.kP1_xy = obj.kP1_xy;
            gains.kP2_xy = obj.kP2_xy;
            gains.kD_xy = obj.kD_xy;
            gains.kP1_psi = obj.kP1_psi;
            gains.kP2_psi = obj.kP2_psi;
            gains.kP1_phi_theta = obj.kP1_phi_theta;
            gains.kP2_phi_theta = obj.kP2_phi_theta;
            gains.kD_phi_theta = obj.kD_phi_theta;
        end
        
        function control = computeControl(obj, state, reference)
            % Main control computation
            % state: [x, y, z, phi, theta, psi, vx, vy, vz, p, q, r]'
            % reference: [xr, yr, zr, psir]'
            
            % Check for NaN in state
            if any(isnan(state)) || any(isinf(state))
                warning('NaN or Inf detected in state!');
                control = [1.35; 0; 0; 0]; % Emergency hover command
                return;
            end
            
            % Extract state
            x = state(1); y = state(2); z = state(3);
            phi = state(4); theta = state(5); psi = state(6);
            vx = state(7); vy = state(8); vz = state(9);
            p = state(10); q = state(11); r = state(12);
            
            % Extract reference
            xr = reference(1); yr = reference(2); 
            zr = reference(3); psir = reference(4);
            
            % ==============================================================
            % OUTER-LOOP CONTROL (Position Control)
            % ==============================================================
            [tau_T, phi_r, theta_r] = obj.outerLoopControl(...
                x, y, z, vx, vy, vz, phi, theta, psi, xr, yr, zr);
            
            % ==============================================================
            % INNER-LOOP CONTROL (Attitude Control)
            % ==============================================================
            [tau_R, tau_P, tau_Y] = obj.innerLoopControl(...
                phi, theta, psi, p, q, r, phi_r, theta_r, psir);
            
            % Check for NaN in control
            control = [tau_T; tau_R; tau_P; tau_Y];
            if any(isnan(control)) || any(isinf(control))
                warning('NaN or Inf detected in control!');
                control = [1.35; 0; 0; 0]; % Emergency hover command
            end
        end
        
        function [tau_T, phi_r, theta_r] = outerLoopControl(obj, ...
                x, y, z, vx, vy, vz, phi, theta, psi, xr, yr, zr)
            % Outer-Loop Position Control (Equations 7-10)
            
            % --- Altitude Control (z-axis) - CORRECTED VERSION ---
            % Position error
            e_z = zr - z;
            
            % Desired velocity from position error (proportional)
            vz_des = obj.kP1_z * e_z;
            
            % Velocity error
            e_vz = vz_des - vz;
            
            % Desired acceleration (proportional on velocity error)
            az_cmd = obj.kP2_z * e_vz;
            
            % DEBUG: Print first few iterations
            persistent iter_count;
            if isempty(iter_count)
                iter_count = 0;
            end
            iter_count = iter_count + 1;
            
            if iter_count <= 3
                fprintf('  [DEBUG] e_z=%.3f, vz_des=%.3f, e_vz=%.3f, az_cmd=%.3f\n', ...
                        e_z, vz_des, e_vz, az_cmd);
            end
            
            % Thrust control (Eq. 7)
            cos_phi_theta = cos(phi) * cos(theta);
            if abs(cos_phi_theta) < 0.1
                cos_phi_theta = 0.1 * sign(cos_phi_theta);
            end
            
            % tau_T = 1 - m*(-g + az_cmd) / (4*Tmax*cos(phi)*cos(theta))
            tau_T = 1 - (obj.m * (-obj.g + az_cmd)) / (4 * obj.Tmax * cos_phi_theta);
            
            if iter_count <= 3
                fprintf('  [DEBUG] tau_T_calc=%.4f\n', tau_T);
            end
            
            % Saturate thrust
            % tau_T = max(0.5, min(2.0, tau_T));
            
            % --- Lateral Position Control (x, y-axis) ---
            % Velocity errors (Eq. 10a, 10b)
            e_dot_x = obj.kP1_xy * (xr - x) - vx;
            e_dot_y = obj.kP1_xy * (yr - y) - vy;
            
            % Derivative of velocity errors
            de_dot_x = (e_dot_x - obj.e_dot_x_prev) / obj.dt;
            de_dot_y = (e_dot_y - obj.e_dot_y_prev) / obj.dt;
            
            % Update previous errors
            obj.e_dot_x_prev = e_dot_x;
            obj.e_dot_y_prev = e_dot_y;
            
            % Virtual control laws (Eq. 10d, 10e)
            vx_cmd = obj.kP2_xy * e_dot_x + obj.kD_xy * de_dot_x;
            vy_cmd = obj.kP2_xy * e_dot_y + obj.kD_xy * de_dot_y;
            
            Vxy = [vx_cmd; vy_cmd];
            
            % Compute desired angles (Eq. 8, 9)
            F_B_star = -(4 * obj.Tmax * (tau_T - 1) / obj.m) * ...
                       [sin(psi), cos(psi); 
                        -cos(psi), sin(psi)];
            
            % Check for singularity and small values
            det_F = abs(det(F_B_star));
            if det_F < 1e-3 || tau_T < 0.05
                % Not enough thrust or singular, set zero desired angles
                phi_r = 0;
                theta_r = 0;
            else
                desired_angles = F_B_star \ Vxy;
                phi_r = desired_angles(1);
                theta_r = desired_angles(2);
                
                % Saturate desired angles
                max_angle = deg2rad(20);  % 20 degrees max tilt
                phi_r = max(-max_angle, min(max_angle, phi_r));
                theta_r = max(-max_angle, min(max_angle, theta_r));
            end
        end
        
        function [tau_R, tau_P, tau_Y] = innerLoopControl(obj, ...
                phi, theta, psi, p, q, r, phi_r, theta_r, psi_r)
            % Inner-Loop Attitude Control with Feedback Linearization
            % (Equations 11-12)

            % Euler rates from angular velocities
            sp = sin(phi); cp = cos(phi);
            st = sin(theta); ct = cos(theta);

            % Avoid gimbal lock - use approximation near singularity
            if abs(ct) < 0.1
                % Near gimbal lock, use simplified approximation
                phi_dot = p + q*sp*sign(st) + r*cp*sign(st);
                theta_dot = q*cp - r*sp;
                psi_dot = (q*sp + r*cp) / max(0.1, abs(ct));
            else
                W_inv = [1,  sp*st/ct,  cp*st/ct;
                         0,  cp,       -sp;
                         0,  sp/ct,     cp/ct];
                eta_dot = W_inv * [p; q; r];
                phi_dot = eta_dot(1);
                theta_dot = eta_dot(2);
                psi_dot = eta_dot(3);
            end

            % --- Roll Control ---
            % Angular error (Eq. 12a)
            e_dot_phi = obj.kP1_phi_theta * (phi_r - phi) - phi_dot;

            % Derivative of angular error
            de_dot_phi = (e_dot_phi - obj.e_dot_phi_prev) / obj.dt;
            obj.e_dot_phi_prev = e_dot_phi;

            % Virtual control (Eq. 12d)
            v_phi = obj.kP2_phi_theta * e_dot_phi + obj.kD_phi_theta * de_dot_phi;

            % --- Pitch Control ---
            % Angular error (Eq. 12b)
            e_dot_theta = obj.kP1_phi_theta * (theta_r - theta) - theta_dot;

            % Derivative of angular error
            de_dot_theta = (e_dot_theta - obj.e_dot_theta_prev) / obj.dt;
            obj.e_dot_theta_prev = e_dot_theta;

            % Virtual control (Eq. 12e)
            v_theta = obj.kP2_phi_theta * e_dot_theta + obj.kD_phi_theta * de_dot_theta;

            % --- Yaw Control ---
            % Angular error (Eq. 12c)
            e_dot_psi = obj.kP1_psi * (psi_r - psi) - psi_dot;

            % Virtual control (Eq. 12f) - P only, no derivative term
            v_psi = obj.kP2_psi * e_dot_psi;

            % --- FEEDBACK LINEARIZATION (Equation 11) ---
            % Compute B and C matrices for Euler-Lagrange formulation
            [B, C] = obj.computeBCmatrices(phi, theta, psi, phi_dot, theta_dot, psi_dot);

            % Virtual control vector
            v = [v_phi; v_theta; v_psi];

            % Euler rate vector
            eta_dot_vec = [phi_dot; theta_dot; psi_dot];

            % Compute W^(-T) (transpose of inverse of W)
            % W relates Euler rates to angular velocities: omega = W^(-1) * eta_dot
            % So W^(-T) = (W^(-1))^T = (W^T)^(-1)
            if abs(ct) < 0.1
                % Near singularity, use simplified approach
                W_inv_T = eye(3);
            else
                W_inv = [1,  sp*st/ct,  cp*st/ct;
                         0,  cp,       -sp;
                         0,  sp/ct,     cp/ct];
                W_inv_T = W_inv';
            end

            % Apply feedback linearization (Eq. 11): M' = W^(-T) * [B*v + C*eta_dot]
            M_prime = W_inv_T * (B * v + C * eta_dot_vec);

            % Extract individual torques
            Mp = M_prime(1);
            Mq = M_prime(2);
            Mr = M_prime(3);

            % Convert torques to normalized control inputs
            tau_R = -Mp / (4 * obj.Tmax * obj.l * sqrt(2) / 2);
            tau_P = -Mq / (4 * obj.Tmax * obj.l * sqrt(2) / 2);
            tau_Y = Mr / (4 * obj.Tmax * obj.cD_cL);

            % Saturate control inputs
            tau_R = max(-1, min(1, tau_R));
            tau_P = max(-1, min(1, tau_P));
            tau_Y = max(-1, min(1, tau_Y));
        end

        function [B, C] = computeBCmatrices(obj, phi, theta, psi, phi_dot, theta_dot, psi_dot)
            % Compute B and C matrices for Euler-Lagrange formulation
            % Based on reference [19]: Martini et al., ICUAS 2022
            %
            % B: Inertia matrix in Euler angle coordinates (3x3)
            % C: Coriolis/centrifugal matrix in Euler angle coordinates (3x3)

            sp = sin(phi); cp = cos(phi);
            st = sin(theta); ct = cos(theta);

            % Avoid division by zero
            if abs(ct) < 0.01
                ct = 0.01 * sign(ct);
            end

            % B Matrix (Inertia transformation from Euler to body frame)
            % This transforms the inertia effects from Euler coordinate space
            % Derived from: I_eta = W^T * I_body * W
            % where W is the matrix relating omega_B to eta_dot

            % For a quadrotor with diagonal inertia I = diag(Ixx, Iyy, Izz)
            % The B matrix accounts for the rotational transformation

            B = zeros(3, 3);
            B(1,1) = obj.Ixx;
            B(1,2) = 0;
            B(1,3) = -obj.Ixx * st;

            B(2,1) = 0;
            B(2,2) = obj.Iyy * cp^2 + obj.Izz * sp^2;
            B(2,3) = (obj.Iyy - obj.Izz) * cp * sp * ct;

            B(3,1) = -obj.Ixx * st;
            B(3,2) = (obj.Iyy - obj.Izz) * cp * sp * ct;
            B(3,3) = obj.Ixx * st^2 + obj.Iyy * sp^2 * ct^2 + obj.Izz * cp^2 * ct^2;

            % C Matrix (Coriolis and centrifugal effects)
            % Contains velocity-dependent terms
            % Derived from Christoffel symbols of the Lagrangian

            C = zeros(3, 3);
            C(1,1) = 0;
            C(1,2) = (obj.Iyy - obj.Izz) * (theta_dot * cp * sp - psi_dot * sp^2 * ct) ...
                     + (obj.Izz - obj.Iyy) * psi_dot * cp^2 * ct - obj.Ixx * psi_dot * ct;
            C(1,3) = (obj.Izz - obj.Iyy) * psi_dot * cp * sp * ct^2;

            C(2,1) = (obj.Izz - obj.Iyy) * (theta_dot * cp * sp - psi_dot * sp^2 * ct) ...
                     + (obj.Iyy - obj.Izz) * psi_dot * cp^2 * ct + obj.Ixx * psi_dot * ct;
            C(2,2) = (obj.Izz - obj.Iyy) * phi_dot * cp * sp;
            C(2,3) = -obj.Ixx * psi_dot * st * ct + obj.Iyy * psi_dot * sp^2 * st * ct ...
                     + obj.Izz * psi_dot * cp^2 * st * ct;

            C(3,1) = (obj.Iyy - obj.Izz) * psi_dot * ct^2 * sp * cp - obj.Ixx * theta_dot * ct;
            C(3,2) = (obj.Izz - obj.Iyy) * (theta_dot * cp * sp * st + phi_dot * sp^2 * ct) ...
                     + (obj.Iyy - obj.Izz) * phi_dot * cp^2 * ct ...
                     + obj.Ixx * psi_dot * st * ct - obj.Iyy * psi_dot * sp^2 * st * ct ...
                     - obj.Izz * psi_dot * cp^2 * st * ct;
            C(3,3) = (obj.Iyy - obj.Izz) * phi_dot * cp * sp * ct^2 ...
                     - obj.Iyy * theta_dot * sp^2 * ct * st ...
                     - obj.Izz * theta_dot * cp^2 * ct * st + obj.Ixx * theta_dot * ct * st;
        end
        
        function resetController(obj)
            % Reset previous error states
            obj.e_dot_x_prev = 0;
            obj.e_dot_y_prev = 0;
            obj.e_dot_phi_prev = 0;
            obj.e_dot_theta_prev = 0;
        end
    end
end
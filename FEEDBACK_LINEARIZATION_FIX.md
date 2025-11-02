# Feedback Linearization Fix - Summary

## Problem Identified

The original implementation in `PDController.m` **was missing the proper feedback linearization** required by Equation 11 of the paper.

### What Was Wrong (Lines 257-266):

```matlab
% --- Simplified torque computation ---
% Use inertia directly without full Euler-Lagrange terms
Mp = obj.Ixx * v_phi;
Mq = obj.Iyy * v_theta;
Mr = obj.Izz * v_psi;
```

This approach **completely omitted**:
- The **B matrix** (inertia transformation in Euler coordinates)
- The **C matrix** (Coriolis/centrifugal effects)
- The **W^(-T)** transformation

### What the Paper Requires (Equation 11, Page 4):

```
M' = W^(-T) * [B*v + C*η̇]
```

Where:
- **W^(-T)**: Transpose of inverse of Euler rate matrix
- **B**: Inertia matrix in Euler angle coordinates (3×3)
- **C**: Coriolis/centrifugal matrix (3×3)
- **v**: Virtual control from PD controller [v_φ, v_θ, v_ψ]^T
- **η̇**: Euler rates [φ̇, θ̇, ψ̇]^T

## What Was Fixed

### 1. Added `computeBCmatrices()` Method

Implemented proper B and C matrices based on Euler-Lagrange formulation:

#### B Matrix (Inertia Transformation):
```matlab
B(1,1) = Ixx
B(1,2) = 0
B(1,3) = -Ixx * sin(θ)

B(2,1) = 0
B(2,2) = Iyy * cos²(φ) + Izz * sin²(φ)
B(2,3) = (Iyy - Izz) * cos(φ) * sin(φ) * cos(θ)

B(3,1) = -Ixx * sin(θ)
B(3,2) = (Iyy - Izz) * cos(φ) * sin(φ) * cos(θ)
B(3,3) = Ixx * sin²(θ) + Iyy * sin²(φ) * cos²(θ) + Izz * cos²(φ) * cos²(θ)
```

#### C Matrix (Coriolis/Centrifugal Effects):
The C matrix contains velocity-dependent terms derived from Christoffel symbols. See lines 339-358 in `PDController.m` for the complete implementation.

### 2. Updated `innerLoopControl()` Method

Now properly applies feedback linearization (lines 257-296):

```matlab
% Compute B and C matrices
[B, C] = obj.computeBCmatrices(phi, theta, psi, phi_dot, theta_dot, psi_dot);

% Virtual control vector
v = [v_phi; v_theta; v_psi];

% Euler rate vector
eta_dot_vec = [phi_dot; theta_dot; psi_dot];

% Compute W^(-T)
W_inv_T = W_inv';

% Apply feedback linearization (Eq. 11)
M_prime = W_inv_T * (B * v + C * eta_dot_vec);

% Extract torques
Mp = M_prime(1);
Mq = M_prime(2);
Mr = M_prime(3);
```

## Why This Matters

### Without Feedback Linearization:
- Controller treats the system as if it has **simple diagonal inertia**
- **Ignores coupling** between roll, pitch, and yaw dynamics
- **Ignores Coriolis effects** from rotating reference frame
- Will have **poor performance** during aggressive maneuvers (like the circular trajectory transitions at t=12.5s and t=32.5s)

### With Feedback Linearization:
- **Cancels nonlinear dynamics** exactly
- Transforms the nonlinear system into a **linear double integrator**: η̈ = v
- PD controller can then easily stabilize the **linearized system**
- Should match the **baseline performance** shown in Figure 6 of the paper

## Expected Results

According to **Table 4** in the paper, the manually tuned controller should achieve:

**Attitude Error Norm RMSE: 12.75 × 10⁻³ rad**

The paper notes (Page 7):
> "The plot of the attitude error norm in Fig. 6 highlights two main areas with higher spikes in correspondence to the quadrotor's change of direction at 12.5s and 32.5s, hence, at the beginning and the end of the circular trajectory."

## How to Test

Run the baseline simulation in MATLAB:

```matlab
>> baselineSim
```

Check the output:
1. **Attitude Error Norm RMSE** should be around **12.75 × 10⁻³ rad**
2. **Figure**: `baseline_attitude_error.png` should match **Figure 6** from the paper
3. **Peak errors** should occur at **t ≈ 12.5s** and **t ≈ 32.5s** (trajectory transitions)

## Technical Details

### Euler-Lagrange Formulation

The kinetic energy in Euler coordinates is:
```
T = (1/2) * η̇^T * D(η) * η̇
```

where `D(η) = W^T * I_body * W` is the inertia matrix in Euler space.

The Euler-Lagrange equations:
```
d/dt(∂L/∂η̇) - ∂L/∂η = τ
```

expand to:
```
D(η)η̈ + C(η, η̇)η̇ = τ
```

### Feedback Linearization Strategy

Choose control law:
```
τ = D(η)v + C(η, η̇)η̇
```

This results in:
```
D(η)η̈ + C(η, η̇)η̇ = D(η)v + C(η, η̇)η̇
D(η)η̈ = D(η)v
η̈ = v  (exact linearization!)
```

Now the PD controller designs `v` for the **linear** system η̈ = v.

## References

- **Paper**: Sönmez et al., "Reinforcement Learning Based Prediction of PID Controller Gains for Quadrotor UAVs"
- **Equation 11** (Page 4): M' = W^(-T)[Bv + Cη̇]
- **Reference [19]**: Martini et al., "Euler-Lagrange Modeling and Control of Quadrotor UAV with Aerodynamic Compensation," ICUAS 2022

## Next Steps for RL Training

Once the baseline performance matches the paper:
1. Use these manually tuned gains as **starting values** (Table 2)
2. Train the DDPG agent to **fine-tune** the inner-loop gains
3. The RL agent will adjust gains **online** to reduce attitude errors further
4. Compare RL-tuned vs. manually-tuned performance (should achieve **RMSE < 12.75 × 10⁻³ rad**)

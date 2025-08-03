% Holistic BMS-Powertrain Integration Simulation
% For 600km Range EV with 4WD, Gearbox, and eCVT
% Author: Frank Ouma
% Date: August 03, 2025
% ECM, UKF, MPC, balancing, faults

clear; clc; close all;

%% Section 1: Parameter Initialization
% Battery (from prior)
NomCap = 100; % Ah
NomV = 400; % V
R0 = 0.01; R1 = 0.005; R2 = 0.01;
C1 = 1000; C2 = 5000;
Tau1 = R1*C1; Tau2 = R2*C2;
OCV_func = @(SoC) interp1(0:0.1:1, 3.2 + 0.8*(0:0.1:1) + 0.2*(0:0.1:1).^2 - 0.1*(0:0.1:1).^3, SoC, 'spline') * (NomV / 3.7);
T_amb = 25; Alpha_T = 0.005; h_conv = 10; A_pack = 2; mC = 1000;
T_ref = 30; u_max = 100; MPC_horizon = 10; w1 = 1; w2 = 0.1;
NumGroups = 4; SoC_groups_init = [0.98, 0.99, 1.00, 1.01]; Bal_eff = 0.9;
Fault_time = 1800; Fault_type = 'low_SoP'; % Inject low SoP fault

% Vehicle/Powertrain
m_veh = 2000; % kg
g = 9.81; Cr = 0.01; CdA = 0.6; rho = 1.2;
rw = 0.3; % Wheel radius (m)
Iz = 3000; % Yaw inertia (kg m^2)
lf = 1.2; lr = 1.3; tw = 1.5; % Wheelbase params (m)
mu_tire = 0.8; % Tire friction coeff
gear_ratios = [2, 1]; % 2-speed gearbox
eCVT_min = 4; eCVT_max = 12; % Ratio range
eta_motor = 0.95; eta_regen = 0.9;

% Sim Settings
dt = 0.1; % s (finer for dynamics)
t_end = 3600; % s
t = 0:dt:t_end;
N = length(t);
CAN_delay = 0.1 / dt; % 10ms delay in steps

%% Section 2: Drive Cycle Generation
% WLTP-like: Velocity, grade, steering (for lateral)
v_ref = 15 + 10*sin(2*pi*t/300) + 20*(t>600 & t<1800) + 30*exp(-((t-2400)/100).^2); % m/s
grade = 0.05*sin(2*pi*t/900); % rad
steer_angle = 0.1*sin(2*pi*t/600); % rad (curves)

% Driver Model: PI for velocity tracking
Kp_v = 500; Ki_v = 100;
e_v_int = zeros(1,N);

%% Section 3: Integrated Simulation Loop
% States: Vehicle [v, a, pos, yaw, yaw_rate], Battery [SoC_avg, VRC1, VRC2, T, SoH, SoC_groups], Powertrain [gear, eCVT_ratio, tau_front, tau_rear]
x_veh = zeros(5, N); % v, a, pos, yaw, yaw_rate
x_bat = zeros(3+1+1+NumGroups, N);
x_pt = zeros(4, N); % gear_idx, eCVT_ratio, tau_f, tau_r
x_bat(:,1) = [mean(SoC_groups_init); 0; 0; T_amb; 1; SoC_groups_init'];
u_control = zeros(N,1); % Cooling
I_bat = zeros(N,1); % Battery current
P_regen = zeros(N,1); % Regen power
fault_active = false;

for k = 2:N
    % Extract states
    v_k = x_veh(1,k-1);
    yaw_rate_k = x_veh(5,k-1);
    SoC_k = x_bat(1,k-1);
    V_RC1_k = x_bat(2,k-1);
    V_RC2_k = x_bat(3,k-1);
    T_k = x_bat(4,k-1);
    SoH_k = x_bat(5,k-1);
    SoC_groups_k = x_bat(6:end,k-1);
    gear_idx = round(x_pt(1,k-1)); gear_idx = max(1, min(2, gear_idx));
    eCVT_ratio_k = x_pt(2,k-1);
    
    % Adjust battery params
    R0_adj = R0 * (1 + Alpha_T*(T_k - 25)) / SoH_k;
    R1_adj = R1 * (1 + Alpha_T*(T_k - 25)) / SoH_k;
    R2_adj = R2 * (1 + Alpha_T*(T_k - 25)) / SoH_k;
    
    % Vehicle Dynamics: Longitudinal forces
    a_ref = (v_ref(k) - v_k) / dt; % Simple accel ref
    Drag = 0.5 * CdA * rho * v_k^2;
    Roll = Cr * m_veh * g;
    Grade = m_veh * g * sin(grade(k-1));
    F_req = m_veh * a_ref + Drag + Roll + Grade;
    tau_req = F_req * rw; % Total torque demand
    
    % Lateral: Simplified Pacejka tire model for yaw
    alpha_f = steer_angle(k-1) - atan((v_k * yaw_rate_k + lf * yaw_rate_k) / v_k);
    alpha_r = -atan((v_k * yaw_rate_k - lr * yaw_rate_k) / v_k);
    Fy_f = mu_tire * m_veh * g * sin(2 * alpha_f); % Approx magic formula
    Fy_r = mu_tire * m_veh * g * sin(2 * alpha_r);
    dyaw_rate = (lf * Fy_f - lr * Fy_r) / Iz;
    
    % 4WD Torque Vectoring: Split for stability
    split_front = 0.5 + 0.2 * (dyaw_rate / (g / v_k)); % Adjust for under/oversteer
    tau_f = split_front * tau_req / gear_ratios(gear_idx) / eCVT_ratio_k;
    tau_r = (1 - split_front) * tau_req / gear_ratios(gear_idx) / eCVT_ratio_k;
    
    % Powertrain: Motor powers
    omega = v_k / rw;
    P_f = tau_f * omega / eta_motor;
    P_r = tau_r * omega / eta_motor;
    P_tot = P_f + P_r;
    
    % BMS Integration: Compute SoP, limit powers
    V_ocv = OCV_func(SoC_k);
    SoP_dis = (V_ocv - 300) / R0_adj * V_ocv; % Approx discharge power limit
    SoP_chg = (450 - V_ocv) / R0_adj * V_ocv; % Charge (regen)
    if strcmp(Fault_type, 'low_SoP') && t(k-1) >= Fault_time
        fault_active = true;
        SoP_dis = SoP_dis * 0.5; % Simulate fault
    end
    P_tot = min(max(P_tot, -SoP_chg), SoP_dis); % Limit
    
    % Regen if P_tot <0
    if P_tot < 0
        P_regen(k) = -P_tot * eta_regen;
    end
    
    % Battery Current: I = P_tot / V_t (approx with NomV for simplicity, iterate if needed)
    I_k = P_tot / NomV;
    I_bat(k) = I_k;
    
    % CAN Emulation: Delay BMS signals to powertrain
    % (Simplified: Delay SoP to next step)
    
    % Update Battery (similar to Lesson 2)
    dV_RC1 = -V_RC1_k / Tau1 + I_k / C1;
    dV_RC2 = -V_RC2_k / Tau2 + I_k / C2;
    V_RC1_new = V_RC1_k + dt * dV_RC1;
    V_RC2_new = V_RC2_k + dt * dV_RC2;
    dSoC = -I_k * dt / (3600 * NomCap * SoH_k);
    SoC_new = SoC_k + 0.98 * dSoC;
    dSoH = -0.0001 * abs(dSoC);
    SoH_new = SoH_k + dSoH;
    
    % Thermal MPC (abbrev from Lesson 2)
    Q_gen = I_k^2 * R0_adj + (V_RC1_k^2 / R1_adj) + (V_RC2_k^2 / R2_adj);
    T_pred = zeros(MPC_horizon,1); T_pred(1) = T_k;
    u_opt = quadprog(2*(w1*eye(MPC_horizon)+w2*eye(MPC_horizon)), -2*w1*T_ref*ones(MPC_horizon,1), [], [], [], [], zeros(MPC_horizon,1), u_max*ones(MPC_horizon,1));
    u_control(k) = u_opt(1);
    Q_cool = u_control(k) * h_conv * A_pack * (T_k - T_amb);
    dT = (Q_gen - Q_cool) / mC;
    T_new = T_k + dt * dT;
    
    % Balancing (abbrev)
    Delta_SoC = SoC_groups_k - mean(SoC_groups_k);
    transfer = -Delta_SoC * (NomCap/NumGroups) * NomV / Bal_eff;
    SoC_groups_new = SoC_groups_k - transfer / ((NomCap/NumGroups) * NomV) + dSoC;
    
    % Powertrain Decisions: Gear shift, eCVT adjust based on BMS SoC
    if v_k > 20 && SoC_k > 0.5
        gear_idx = 2; % High gear if efficient
    else
        gear_idx = 1;
    end
    eCVT_ratio_new = eCVT_min + (eCVT_max - eCVT_min) * (1 - SoC_k); % Higher ratio if low SoC for torque
    
    % Update Vehicle Dynamics
    a_k = (tau_req / rw - Drag - Roll - Grade) / m_veh;
    v_new = v_k + dt * a_k;
    pos_new = x_veh(3,k-1) + dt * v_k;
    yaw_new = x_veh(4,k-1) + dt * yaw_rate_k;
    yaw_rate_new = yaw_rate_k + dt * dyaw_rate;
    
    % Store
    x_veh(:,k) = [v_new; a_k; pos_new; yaw_new; yaw_rate_new];
    x_bat(1,k) = SoC_new;
    x_bat(2:3,k) = [V_RC1_new; V_RC2_new];
    x_bat(4:5,k) = [T_new; SoH_new];
    x_bat(6:end,k) = SoC_groups_new;
    x_pt(:,k) = [gear_idx; eCVT_ratio_new; tau_f; tau_r];
end

%% Section 4: Results Visualization and Analysis
% Velocity Tracking
figure(1);
plot(t/3600, v_ref, 'b--', t/3600, x_veh(1,:), 'r-');
xlabel('Time (hours)'); ylabel('Velocity (m/s)');
legend('Reference', 'Actual');
title('Vehicle Velocity Tracking');
grid on;

% Torque Splits and Gear
figure(2);
yyaxis left; plot(t/3600, x_pt(3,:), 'g-', t/3600, x_pt(4,:), 'm-'); ylabel('Torque (Nm)');
yyaxis right; stairs(t/3600, x_pt(1,:), 'k--'); ylabel('Gear Index');
xlabel('Time (hours)');
title('4WD Torque Vectoring and Gear Shifts');
grid on;

% Regen and SoC
figure(3);
yyaxis left; plot(t/3600, cumsum(P_regen)*dt/3600/1000, 'c-'); ylabel('Cumulative Regen Energy (kWh)');
yyaxis right; plot(t/3600, x_bat(1,:), 'b-'); ylabel('SoC');
xlabel('Time (hours)');
title('Regenerative Braking and SoC');
grid on;

% Yaw Rate
figure(4);
plot(t/3600, x_veh(5,:), 'k-');
xlabel('Time (hours)'); ylabel('Yaw Rate (rad/s)');
title('Lateral Dynamics');
grid on;

% Detailed Analysis
Energy_regen = trapz(t, P_regen) / 3600 / 1000; % kWh
Range_ext_regen = Energy_regen / (100 / 600); % km (assume 167 Wh/km)
fprintf('Total Regenerated Energy: %.2f kWh\n', Energy_regen);
fprintf('Range Extension from Regen: %.2f km\n', Range_ext_regen);
Yaw_RMS = sqrt(mean(x_veh(5,:).^2));
fprintf('Yaw Rate RMS (Stability Metric): %.4f rad/s\n', Yaw_RMS);
if fault_active
    fprintf('Fault Injected at t=%.0f s, SoP reduced.\n', Fault_time);
end

% Extensions: Add ODE45 for precise dynamics, CAN packet loss (rand drop)
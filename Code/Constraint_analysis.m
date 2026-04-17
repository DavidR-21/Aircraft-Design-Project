% Constraint plot for aircraft sizing
% Includes takeoff, cruise, instant turn, sustained turn, stall, and landing constraints

% Variable Initiation
syms WS
rho_op = 0.000889;       % air density at 35000 ft [slug/ft^3]
rho_SL = 0.0023769;      % sea-level air density [slug/ft^3]

% Takeoff, stall, and landing parameters
C_Lmax = 1.8;            % maximum lift coefficient
V_stall = 110;         % stall velocity in ft/s
V_LOF = V_stall*1.2;     % lift-off speed (takeoff speed) in ft/s
q_sl = 0.5*rho_SL*V_LOF^2; % dynamic pressure at takeoff

% Cruise Parameters
V_op = 440;              % operating velocity at cruise in ft/s
np = 0.85;               % propeller efficiency
q_op = 0.5*rho_op*V_op^2; % dynamic pressure at operating conditions
C_D = 0.0119;            % drag coefficient at cruise
W1W0 = 4800/5300;        % weight fraction at cruise (current weight / takeoff weight)
P1PSl = 1.9;             % power scaling at cruise relative to sea-level

% Instant Turn Parameters
V_in = 350;              % velocity at instantaneous turn in ft/s
q_in = 0.5*rho_op*V_in^2; % dynamic pressure for instantaneous turn
k2 = 0.059;              % constant for induced drag in instant turn equation
n1 = 2;                  % load factor for instantaneous turn (conservative)
W2W0 = .9;                % weight fraction at instant turn (conservative)
P2PSL = 1.9;             % power fraction at instant turn

% Sustained Turn Parameters
V_sus = 400;             % velocity for sustained turn in ft/s
q_sus = 0.5*rho_op*V_sus^2; % dynamic pressure for sustained turn
C_D0 = 0.008;            % zero-lift drag coefficient
k3 = k2;                 % constant for sustained turn induced drag
n2 = 1.5;                  % load factor for sustained turn
W3W0 = 5000/5300;        % weight fraction for sustained turn (conservative)
P3PSL = 1.9;             % power fraction for sustained turn

% Define WS range for plotting
WS_range = [10 70];      % wing loading range [lb/ft²]

% Equations as anonymous functions (W/P constraints)
WP1_fun = @(WS) (V_LOF/np)*((q_sl*C_D0)./WS + 32.2/V_LOF);          % Takeoff constraint
WP4_fun = @(WS) (V_op/np)*((q_op*C_D)./WS)*W1W0*P1PSl;               % Cruise constraint
WP5_fun = @(WS) (V_in/np)*((q_in*C_D)./WS + k2*n1^2*WS/q_in)*W2W0*P2PSL; % Instant Turn
WP6_fun = @(WS) (V_sus/np)*((q_sus*C_D0)./WS + k3*n2^2*WS/q_sus)*W3W0*P3PSL; % Sustained Turn

% W/S constraints (vertical lines for stall and landing)
WS_stall = 0.5*rho_SL*V_stall^2*C_Lmax;        % stall limit (max wing loading to avoid stall)
WS_landing = 0.5*rho_SL*(1.3*V_stall)^2*C_Lmax; % landing limit (max wing loading for landing)

% Plot
figure; hold on;

% Full WS range for plotting constraint lines
WS_full = linspace(5, WS_landing, 500); % full axis for curves

% Compute W/P for each constraint
WP1_full = WP1_fun(WS_full);
WP4_full = WP4_fun(WS_full);
WP5_full = WP5_fun(WS_full);
WP6_full = WP6_fun(WS_full);

% Feasible region WS range (left of stall)
WS_vals = linspace(5, WS_stall, 500);

% Compute W/P for feasible region
WP1_vals = WP1_fun(WS_vals);
WP4_vals = WP4_fun(WS_vals);
WP5_vals = WP5_fun(WS_vals);
WP6_vals = WP6_fun(WS_vals);

% Maximum W/P at each WS for shading
WP_lim = max([WP1_vals; WP4_vals; WP5_vals; WP6_vals], [], 1);

figure; hold on;

% Shade feasible region (above curves, left of WS_stall)
fill([WS_vals fliplr(WS_vals)], ...
     [WP_lim repmat(max(WP_lim)*1.5,1,length(WS_vals))], ...
     [0.9 1 0.9], 'FaceAlpha',0.4, 'EdgeColor','none');

% Plot constraint lines over full WS range
plot(WS_full, WP1_full, 'r-', 'LineWidth',1.5); % Takeoff
plot(WS_full, WP4_full, 'g-', 'LineWidth',1.5); % Cruise
plot(WS_full, WP5_full, 'm-', 'LineWidth',1.5); % Instant Turn
plot(WS_full, WP6_full, 'y-', 'LineWidth',1.5); % Sustained Turn

% Vertical lines for stall and landing
xline(WS_stall, 'c-', 'Stall');
xline(WS_landing, 'b-', 'Landing');

xlabel('Wing Loading (WS) [lb/ft²]');
ylabel('Weight Per Unit Power (W/P) [lb/hp]');
title('Aircraft Constraint Plot with Feasible Region');
legend('Feasible Region','Takeoff','Cruise','Instant Turn','Sustained Turn','Stall','Landing');
grid on;
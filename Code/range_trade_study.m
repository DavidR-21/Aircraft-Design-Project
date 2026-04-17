clc
clear
close all

%Inputs 

% Payload weight (passengers + luggage)
W_payload = 1300;          % lb

% Cruise speed
V = 440;                  % ft/s (~270 knots)

% Lift-to-drag ratio
L_D = 13;                 % typical turboprop VLJ

% Loiter time
E_loiter = 1800;          % seconds (30 minutes)

% Specific Fuel Consumption
SFC_cruise = 0.6/3600;    % 1/s
SFC_loiter = 0.7/3600;    % 1/s

% Empty weight model (Raymer)
A = 2.36;                 % coefficient
C = -0.18;                % exponent

% Reserve fuel fraction
reserve_fraction = 0.06;

% Mission segment fixed weight fractions
wf_warmup_takeoff = 0.97;
wf_climb          = 0.985;
wf_descent_land   = 0.995;

%Trade study Parameters

% Range vector (ft)
R_min = 500*6076;  % 1000 nmi in ft
R_max = 5000*6076;  % 5000 nmi in ft
num_points = 30;    % number of points in trade study
R_cruise_vec = linspace(R_min,R_max,num_points);


W0 = zeros(size(R_cruise_vec));
We = zeros(size(R_cruise_vec));
Wf = zeros(size(R_cruise_vec));
We_frac = zeros(size(R_cruise_vec));
Wf_frac = zeros(size(R_cruise_vec));

% Trade study loop

for i = 1:length(R_cruise_vec)
    
    R_cruise = R_cruise_vec(i);  % current cruise range

    % Cruise weight fraction (Breguet range)
    wf_cruise = exp(-(R_cruise*SFC_cruise)/(V*L_D));

    % Loiter weight fraction (Breguet endurance)
    wf_loiter = exp(-(E_loiter*SFC_loiter)/(L_D));

    % Total mission fraction
    wf_mission = wf_warmup_takeoff * wf_climb * wf_cruise * wf_loiter * wf_descent_land;

    % Total fuel fraction including reserves
    fuel_fraction = (1 - wf_mission) + reserve_fraction;

    % ITERATIVE SOLVER FOR TAKEOFF WEIGHT
    W0_guess = 10000;   % initial guess
    tolerance = 1;      % convergence tolerance
    error = 1000;

    while error > tolerance
        We_guess = A * W0_guess^C * W0_guess;
        Wf_guess = fuel_fraction * W0_guess;
        W0_new = We_guess + Wf_guess + W_payload;
        error = abs(W0_new - W0_guess);
        W0_guess = W0_new;
    end

    % Store results
    W0(i) = W0_new;
    We(i) = We_guess;
    Wf(i) = Wf_guess;
    We_frac(i) = We_guess / W0_new;
    Wf_frac(i) = Wf_guess / W0_new;
end

%Plots  

% Plot weight fractions
figure;
plot(R_cruise_vec/6076, We_frac,'b-', 'LineWidth',2); hold on;
plot(R_cruise_vec/6076, Wf_frac,'r-', 'LineWidth',2);
grid on
xlabel('Cruise Range (nmi)')
ylabel('Weight Fraction')
title('Trade Study: Weight Fractions vs Cruise Range')
legend('Empty Weight Fraction W_e/W_0','Fuel Weight Fraction W_f/W_0')

% Plot absolute weights
figure;
plot(R_cruise_vec/6076, W0,'k-','LineWidth',2); hold on
plot(R_cruise_vec/6076, We,'b-','LineWidth',2);
plot(R_cruise_vec/6076, Wf,'r-','LineWidth',2);
plot(R_cruise_vec/6076, W_payload*ones(size(R_cruise_vec)),'g--','LineWidth',2);
grid on
xlabel('Cruise Range (nmi)')
ylabel('Weight (lb)')
title('Trade Study: Aircraft Weights vs Cruise Range')
legend('Takeoff Weight W_0','Empty Weight W_e','Fuel Weight W_f','Payload Weight')
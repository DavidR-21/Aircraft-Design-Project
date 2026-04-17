
% Payload weight 
W_payload = 1300;  % lb

% Cruise range
R_cruise = 6.68e6; % ft 

% Cruise velocity
V = 440;           % ft/s 

% L/D ratio
L_D = 13;              

% Loiter time
E_loiter = 1800;  % seconds


% Specific Fuel Consumption
SFC_cruise = 0.6/3600;    % 1/s 
SFC_loiter = 0.7/3600;    % 1/s

% initial guess
W0_guess = 8000;    

% Empty weight 
A = 2.36;              
C = -0.18;             

wf_warmup_takeoff = 0.97;
wf_climb          = 0.985;
wf_descent_land   = 0.995;

wf_cruise = exp(-(R_cruise*SFC_cruise)/(V*L_D));

wf_loiter = exp(-(E_loiter*SFC_loiter)/(L_D));

wf_mission = wf_warmup_takeoff * wf_climb * wf_cruise * wf_loiter * wf_descent_land;

fuel_fraction = 1.06*(1 - wf_mission);



tolerance = 1;       
error = 1000;

while error > tolerance

    % Empty weight estimate
    We = A*(W0_guess^C)*W0_guess;

    % Fuel weight
    Wf = fuel_fraction * W0_guess;

    % New takeoff weight
    W0_new = We + Wf + W_payload;

    % convergence check
    error = abs(W0_new - W0_guess);

    % update guess
    W0_guess = W0_new;

end



fprintf('CONVERGED AIRCRAFT WEIGHTS\n')
fprintf('Takeoff Weight W0 = %.2f lb\n',W0_new)
fprintf('Empty Weight  We = %.2f lb\n',We)
fprintf('Fuel Weight   Wf = %.2f lb\n',Wf)
fprintf('Payload       = %.2f lb\n',W_payload)

fprintf('\nMission Fuel Fraction = %.3f\n',fuel_fraction)
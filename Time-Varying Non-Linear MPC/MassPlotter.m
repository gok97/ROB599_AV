%% Prepare Workspace
clear
clc

%% Prepare variables
Ttot = 20
dt = 0.1
syms t_m
m0 = 0.65;
delta_mass = m0*0.25;
t_drop = 10; % Time of descrete drop
pw = 18; % Prediction window

m = m0+ delta_mass -(1/(1+exp(-10000*(t_m-t_drop)))*delta_mass);


%% Loop through all
for k = 1:(Ttot/dt)
    t = linspace(k*dt, (k+pw-1)*dt,pw);
    m_current = double(subs(m, t_m, t(1)));
    mHistory(k) = m_current;
end

%% Plot
T_series = 0:dt:Ttot;
T_series = T_series(1:end-1)
plot(T_series, mHistory);
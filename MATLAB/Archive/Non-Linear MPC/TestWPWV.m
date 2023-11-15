%% Prepare workspace
clear
clc

%% Prepare variables
tot_inc = 20;
wp=[0,0,0;
    0,0,1;
    2.5,2.5,1
    5,5,1;
    5,5,0.1]
wv=[0,0,0;
    0,0,0;
    10,10,0;
    0,0,0;
    0,0,0]

%% Execute the trajectory
xdesired = QuadrotorRawTrajectory(tot_inc, wp, wv)
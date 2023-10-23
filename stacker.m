%% Create the stacker function
function eom = stacker(tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3)
    % Create variables
    syms dt Ix Iy Iz Ax Ay Az kdx kdy kdz xdot_w ydot_w zdot_w l kf km ka m g

    % Define the states
    syms x y z u v w phi theta psy p q r 
    
    % Define the inputs
    syms w1 w2 w3 w4

    eom(1) = tk1;
    eom(2) = tk2;
    eom(3) = tk3;
    eom(4) = td1;
    eom(5) = td2;
    eom(6) = td3;
    eom(7) = rk1;
    eom(8) = rk2;
    eom(9) = rk3;
    eom(10) = rd1;
    eom(11) = rd2;
    eom(12) = rd3;
end
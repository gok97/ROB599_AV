%% Define the linearization function
function [tk1_function, tk2_function, tk3_function, td1_function, td2_function, td3_function, rk1_function, rk2_function, rk3_function, rd1_function, rd2_function, rd3_function] = Nonlinearizer(tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3)
    
    % Define the symbolic variables
    syms x y z xdot ydot zdot phi theta psy p q r 
    
    % Define the inputs
    syms w1 w2 w3 w4

    % Convert the functions into function handles
    tk1_function = matlabFunction(tk1, 'Vars', [x, y, z, xdot, ydot, zdot, phi, theta, psy, p, q, r, w1, w2, w3, w4]);
    tk2_function = matlabFunction(tk2, 'Vars', [x, y, z, xdot, ydot, zdot, phi, theta, psy, p, q, r, w1, w2, w3, w4]);
    tk3_function = matlabFunction(tk3, 'Vars', [x, y, z, xdot, ydot, zdot, phi, theta, psy, p, q, r, w1, w2, w3, w4]);
    td1_function = matlabFunction(td1, 'Vars', [x, y, z, xdot, ydot, zdot, phi, theta, psy, p, q, r, w1, w2, w3, w4]);
    td2_function = matlabFunction(td2, 'Vars', [x, y, z, xdot, ydot, zdot, phi, theta, psy, p, q, r, w1, w2, w3, w4]);
    td3_function = matlabFunction(td3, 'Vars', [x, y, z, xdot, ydot, zdot, phi, theta, psy, p, q, r, w1, w2, w3, w4]);
    rk1_function = matlabFunction(rk1, 'Vars', [x, y, z, xdot, ydot, zdot, phi, theta, psy, p, q, r, w1, w2, w3, w4]);
    rk2_function = matlabFunction(rk2, 'Vars', [x, y, z, xdot, ydot, zdot, phi, theta, psy, p, q, r, w1, w2, w3, w4]);
    rk3_function = matlabFunction(rk3, 'Vars', [x, y, z, xdot, ydot, zdot, phi, theta, psy, p, q, r, w1, w2, w3, w4]);
    rd1_function = matlabFunction(rd1, 'Vars', [x, y, z, xdot, ydot, zdot, phi, theta, psy, p, q, r, w1, w2, w3, w4]);
    rd2_function = matlabFunction(rd2, 'Vars', [x, y, z, xdot, ydot, zdot, phi, theta, psy, p, q, r, w1, w2, w3, w4]);
    rd3_function = matlabFunction(rd3, 'Vars', [x, y, z, xdot, ydot, zdot, phi, theta, psy, p, q, r, w1, w2, w3, w4]);

end
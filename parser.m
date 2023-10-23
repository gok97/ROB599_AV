%% Create the stacker function
function  [x_out, y_out, z_out, u_out, v_out, w_out, phi_out, theta_out, psy_out, p_out, q_out, r_out, w1_out, w2_out, w3_out, w4_out] = parser(X, U)

    % Create variables
    syms dt Ix Iy Iz Ax Ay Az kdx kdy kdz xdot_w ydot_w zdot_w l kf km ka m g

    % Define the states
    syms x y z u v w phi theta psy p q r 
    
    % Define the inputs
    syms w1 w2 w3 w4

    x_out = X(1);
    y_out = X(2);
    z_out = X(3);
    u_out = X(4);
    v_out = X(5);
    w_out = X(6);
    phi_out = X(7);
    theta_out = X(8);
    psy_out = X(9);
    p_out = X(10);
    q_out = X(11);
    r_out = X(12);
    w1_out = U(1);
    w2_out = U(2);
    w3_out = U(3);
    w4_out = U(4);

end

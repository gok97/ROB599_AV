%% Define the ODE
function X_dot = NonlinearizedDrone(T, X, tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3)

    % Define the symbolic variables
    syms x y z u v w phi theta psy p q r 
    
    % Define the inputs
    syms w1 w2 w3 w4

    % Write the state equation
    U = [800, 800, 800, 800]';

    % Substitute the inputs and associate the state vector
    X_dot(1) = double(subs(tk1, [x; y; z; u; v; w; phi; theta; psy; p; q; r; w1; w2; w3; w4], [X(1); X(2); X(3); X(4); X(5); X(6); X(7); X(8); X(9); X(10); X(11); X(12); U(1); U(2); U(3); U(4)]));
    X_dot(2) = double(subs(tk2, [x; y; z; u; v; w; phi; theta; psy; p; q; r; w1; w2; w3; w4], [X(1); X(2); X(3); X(4); X(5); X(6); X(7); X(8); X(9); X(10); X(11); X(12); U(1); U(2); U(3); U(4)]));
    X_dot(3) = double(subs(tk3, [x; y; z; u; v; w; phi; theta; psy; p; q; r; w1; w2; w3; w4], [X(1); X(2); X(3); X(4); X(5); X(6); X(7); X(8); X(9); X(10); X(11); X(12); U(1); U(2); U(3); U(4)]));
    X_dot(4) = double(subs(td1, [x; y; z; u; v; w; phi; theta; psy; p; q; r; w1; w2; w3; w4], [X(1); X(2); X(3); X(4); X(5); X(6); X(7); X(8); X(9); X(10); X(11); X(12); U(1); U(2); U(3); U(4)]));
    X_dot(5) = double(subs(td2, [x; y; z; u; v; w; phi; theta; psy; p; q; r; w1; w2; w3; w4], [X(1); X(2); X(3); X(4); X(5); X(6); X(7); X(8); X(9); X(10); X(11); X(12); U(1); U(2); U(3); U(4)]));
    X_dot(6) = double(subs(td3, [x; y; z; u; v; w; phi; theta; psy; p; q; r; w1; w2; w3; w4], [X(1); X(2); X(3); X(4); X(5); X(6); X(7); X(8); X(9); X(10); X(11); X(12); U(1); U(2); U(3); U(4)]));
    X_dot(7) = double(subs(rk1, [x; y; z; u; v; w; phi; theta; psy; p; q; r; w1; w2; w3; w4], [X(1); X(2); X(3); X(4); X(5); X(6); X(7); X(8); X(9); X(10); X(11); X(12); U(1); U(2); U(3); U(4)]));
    X_dot(8) = double(subs(rk2, [x; y; z; u; v; w; phi; theta; psy; p; q; r; w1; w2; w3; w4], [X(1); X(2); X(3); X(4); X(5); X(6); X(7); X(8); X(9); X(10); X(11); X(12); U(1); U(2); U(3); U(4)]));
    X_dot(9) = double(subs(rk3, [x; y; z; u; v; w; phi; theta; psy; p; q; r; w1; w2; w3; w4], [X(1); X(2); X(3); X(4); X(5); X(6); X(7); X(8); X(9); X(10); X(11); X(12); U(1); U(2); U(3); U(4)]));
    X_dot(10) = double(subs(rd1, [x; y; z; u; v; w; phi; theta; psy; p; q; r; w1; w2; w3; w4], [X(1); X(2); X(3); X(4); X(5); X(6); X(7); X(8); X(9); X(10); X(11); X(12); U(1); U(2); U(3); U(4)]));
    X_dot(11) = double(subs(rd2, [x; y; z; u; v; w; phi; theta; psy; p; q; r; w1; w2; w3; w4], [X(1); X(2); X(3); X(4); X(5); X(6); X(7); X(8); X(9); X(10); X(11); X(12); U(1); U(2); U(3); U(4)]));
    X_dot(12) = double(subs(rd3, [x; y; z; u; v; w; phi; theta; psy; p; q; r; w1; w2; w3; w4], [X(1); X(2); X(3); X(4); X(5); X(6); X(7); X(8); X(9); X(10); X(11); X(12); U(1); U(2); U(3); U(4)]));

    X_dot = X_dot';
end 
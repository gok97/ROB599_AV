%% Define the linearization function
function [A, B] = Linearizer(constants, equilibrium, debug)
    
    % Create variables
    syms Ix Iy Iz Ax Ay Az kdx kdy kdz xdot_w ydot_w zdot_w l kf km ka m g

    % Define the states
    syms x y z u v w phi theta psy p q r 
    
    % Define the states derivative
    syms x_d y_d z_d u_d v_d w_d phi_d theta_d psi_d p_d q_d r_d 
    
    % Define the inputs
    syms w1 w2 w3 w4
    
    % Define the equations
    tk1 = (cos(theta)*cos(psy))*u + ((-sin(psy)*cos(phi))+(cos(psy)*sin(theta)*sin(phi)))*v + ((sin(phi)*sin(psy))+(cos(phi)*sin(theta)*cos(psy)))*w;
    tk2 = (cos(theta)*sin(psy))*u + ((cos(psy)*cos(phi))+(sin(psy)*sin(theta)*sin(phi)))*v + ((-sin(phi)*cos(psy))+(cos(phi)*sin(theta)*sin(psy)))*w;
    tk3 = (-sin(theta))*u + (sin(phi)*cos(theta))*v + (cos(phi)*cos(theta))*w;

    td1 = r*v - q*w + ((m*g*sin(theta)) - (ka*Ax*((xdot_w-u)^2)))/m;
    td2 = p*w - r*u + ((-m*g*sin(phi)*cos(theta)) - (ka*Ay*((ydot_w-v)^2)))/m;
    td3 = q*u - p*v + ((-m*g*cos(phi)*cos(theta)) + (kf*(w1^2 + w2^2 + w3^2 + w4^2)) - (ka*Az*((zdot_w-w)^2)))/m;
    
    rk1 = p + q*sin(phi)*tan(theta) + r*cos(psy)*tan(theta);
    rk2 = q*cos(phi) - r*sin(phi);
    rk3 = q*sin(phi)*sec(theta) + r*cos(phi)*sec(theta);
    
    rd1 = (-((Iz-Iy)*q*r) - q*(w1+w2+w3+w4) - kdx*p + l*kf*(-w1^2 - w2^2 + w3^2 + w4^2))/Ix;
    rd2 = (-((Ix-Iz)*p*r) + p*(w1+w2+w3+w4) - kdy*q + l*kf*(-w1^2 + w2^2 + w3^2 - w4^2))/Iy;
    rd3 = (-((Iy-Ix)*p*q) - kdz*r + km*(w1^2 + w2^2 + w3^2 + w4^2))/Iz;
    
    % Calculate the jacobian
    Ja = jacobian([tk1; tk2; tk3; td1; td2; td3; rk1; rk2; rk3; rd1; rd2; rd3], [x; y; z; u; v; w; phi; theta; psy; p; q; r ]);
    Jb = jacobian([tk1; tk2; tk3; td1; td2; td3; rk1; rk2; rk3; rd1; rd2; rd3], [w1; w2; w3; w4 ]);
    
    % Print when debugging
    if debug == true
        disp("The symbolic jacobians are:")
        disp(Ja)
        disp(Jb)
    end

    % Check if any constants have been passed
    if isempty(constants)
        updated_Ja = Ja;
        updated_Jb = Jb;     

    else
        % Substitute constant values
        Ja_substituted = (subs(Ja, [Ix; Iy; Iz; Ax; Ay; Az; kdx; kdy; kdz; xdot_w; ydot_w; zdot_w; l; kf; km; ka; m; g], constants));
        Jb_substituted = (subs(Jb, [Ix; Iy; Iz; Ax; Ay; Az; kdx; kdy; kdz; xdot_w; ydot_w; zdot_w; l; kf; km; ka; m; g], constants));
    
        % Print when debugging
        if debug == true
            disp("The jacobians with substituted constants are:")
            disp(Ja_substituted)
            disp(Jb_substituted)
        end

        updated_Ja = Ja_substituted;
        updated_Jb = Jb_substituted;
    end

    % Check if an equilibrium has been passed
    if isempty(equilibrium)
        A = updated_Ja;
        B = updated_Jb;
    else
        % Evaluate at equilibria
        Ja_equilibrium = (subs(updated_Ja, [x; y; z; u; v; w; phi; theta; psy; p; q; r; w1; w2; w3; w4], equilibrium));
        Jb_equilibrium = (subs(updated_Jb, [x; y; z; u; v; w; phi; theta; psy; p; q; r; w1; w2; w3; w4], equilibrium));
        
        % Print when debugging
        if debug == true
            disp("The jacobians evaluated at the operating point are:")
            disp(Ja_equilibrium)
            disp(Jb_equilibrium)
        end

        A = Ja_equilibrium;
        B = Jb_equilibrium;

    end
        

end
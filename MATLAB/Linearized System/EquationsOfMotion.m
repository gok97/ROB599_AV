%% Define the linearization function
function [tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3] = EquationsOfMotion(constants, debug)
    
    % Create variables
    syms Ix Iy Iz Ax Ay Az kdx kdy kdz xdot_w ydot_w zdot_w l kf km ka m g

    % Define the states
    syms x y z xdot ydot zdot phi theta psy p q r
    
    % Define the inputs
    syms w1 w2 w3 w4
    
    % Define the equations
    tk1_sym = xdot;
    tk2_sym = ydot;
    tk3_sym = zdot;
    
    td1_sym = ((kf*(w1^2 + w2^2 + w3^2 + w4^2))*(sin(phi)*sin(psy)+cos(phi)*sin(theta)*cos(psy)) - (ka*Ax*((xdot_w-xdot)^2)))/m;
    td2_sym = ((kf*(w1^2 + w2^2 + w3^2 + w4^2))*(-sin(phi)*cos(psy)+sin(psy)*sin(theta)*cos(phi))  - (ka*Ay*((ydot_w-ydot)^2)))/m;
    td3_sym = (-m*g + (kf*(w1^2 + w2^2 + w3^2 + w4^2))*(cos(phi)*cos(theta))  - (ka*Az*((zdot_w-zdot)^2)))/m;

    rk1_sym = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
    rk2_sym = q*cos(phi) - r*sin(phi);
    rk3_sym = q*sin(phi)*sec(theta) + r*cos(phi)*sec(theta);
    
    % rd1_sym = (-((Iz-Iy)*q*r) - q*(w1+w2+w3+w4) - kdx*p + l*kf*(-w1^2 - w2^2 + w3^2 + w4^2))/Ix; THIS IS WRONG
    % rd2_sym = (-((Ix-Iz)*p*r) + p*(w1+w2+w3+w4) - kdy*q + l*kf*(-w1^2 + w2^2 + w3^2 - w4^2))/Iy; THIS IS WRONG
    % rd3_sym = (-((Iy-Ix)*p*q) - kdz*r + km*(w1^2 - w2^2 + w3^2 - w4^2))/Iz; THIS IS WRONG
    rd1_sym = (-((Iz-Iy)*q*r)  - kdx*p + l*kf*(-w1^2 - w2^2 + w3^2 + w4^2))/Ix;
    rd2_sym = (-((Ix-Iz)*p*r)  - kdy*q + l*kf*(-w1^2 + w2^2 + w3^2 - w4^2))/Iy;
    rd3_sym = (-((Iy-Ix)*p*q) - kdz*r + km*(w1^2 - w2^2 + w3^2 - w4^2))/Iz;
  
    
    % Print when debugging
    if debug == true
        disp("Symbolic Translational Kinematics:")
        disp(tk1_sym)
        disp(tk2_sym)
        disp(tk3_sym)
        disp("Symbolic Translational Dynamics:")
        disp(td1_sym)
        disp(td2_sym)
        disp(td3_sym)
        disp("Symbolic Rotational Kinematics:")
        disp(rk1_sym)
        disp(rk2_sym)
        disp(rk3_sym)
        disp("Symbolic Rotational Dynamics:")
        disp(rd1_sym)
        disp(rd2_sym)
        disp(rd3_sym)
    end

    % Check if any constants have been passed
    if isempty(constants)
        tk1 = tk1_sym;
        tk2 = tk2_sym;
        tk3 = tk3_sym;
        td1 = td1_sym;
        td2 = td2_sym;
        td3 = td3_sym;
        rk1 = rk1_sym;
        rk2 = rk2_sym;
        rk3 = rk3_sym;
        rd1 = rd1_sym;
        rd2 = rd2_sym;
        rd3 = rd3_sym;

    else
        % Substitute constant values
        tk1_substituted = (subs(tk1_sym, [Ix; Iy; Iz; Ax; Ay; Az; kdx; kdy; kdz; xdot_w; ydot_w; zdot_w; l; kf; km; ka; m; g], constants));
        tk2_substituted = (subs(tk2_sym, [Ix; Iy; Iz; Ax; Ay; Az; kdx; kdy; kdz; xdot_w; ydot_w; zdot_w; l; kf; km; ka; m; g], constants));
        tk3_substituted = (subs(tk3_sym, [Ix; Iy; Iz; Ax; Ay; Az; kdx; kdy; kdz; xdot_w; ydot_w; zdot_w; l; kf; km; ka; m; g], constants));
        td1_substituted = (subs(td1_sym, [Ix; Iy; Iz; Ax; Ay; Az; kdx; kdy; kdz; xdot_w; ydot_w; zdot_w; l; kf; km; ka; m; g], constants));
        td2_substituted = (subs(td2_sym, [Ix; Iy; Iz; Ax; Ay; Az; kdx; kdy; kdz; xdot_w; ydot_w; zdot_w; l; kf; km; ka; m; g], constants));
        td3_substituted = (subs(td3_sym, [Ix; Iy; Iz; Ax; Ay; Az; kdx; kdy; kdz; xdot_w; ydot_w; zdot_w; l; kf; km; ka; m; g], constants));
        rk1_substituted = (subs(rk1_sym, [Ix; Iy; Iz; Ax; Ay; Az; kdx; kdy; kdz; xdot_w; ydot_w; zdot_w; l; kf; km; ka; m; g], constants));
        rk2_substituted = (subs(rk2_sym, [Ix; Iy; Iz; Ax; Ay; Az; kdx; kdy; kdz; xdot_w; ydot_w; zdot_w; l; kf; km; ka; m; g], constants));
        rk3_substituted = (subs(rk3_sym, [Ix; Iy; Iz; Ax; Ay; Az; kdx; kdy; kdz; xdot_w; ydot_w; zdot_w; l; kf; km; ka; m; g], constants));
        rd1_substituted = (subs(rd1_sym, [Ix; Iy; Iz; Ax; Ay; Az; kdx; kdy; kdz; xdot_w; ydot_w; zdot_w; l; kf; km; ka; m; g], constants));
        rd2_substituted = (subs(rd2_sym, [Ix; Iy; Iz; Ax; Ay; Az; kdx; kdy; kdz; xdot_w; ydot_w; zdot_w; l; kf; km; ka; m; g], constants));
        rd3_substituted = (subs(rd3_sym, [Ix; Iy; Iz; Ax; Ay; Az; kdx; kdy; kdz; xdot_w; ydot_w; zdot_w; l; kf; km; ka; m; g], constants));
    
        % Print when debugging
        if debug == true
        disp("Translational Kinematics w/ Substituted Constants:")
        disp(tk1_substituted)
        disp(tk2_substituted)
        disp(tk3_substituted)
        disp("Translational Dynamics w/ Substituted Constants:")
        disp(td1_substituted)
        disp(td2_substituted)
        disp(td3_substituted)
        disp("Rotational Kinematics w/ Substituted Constants:")
        disp(rk1_substituted)
        disp(rk2_substituted)
        disp(rk3_substituted)
        disp("Rotational Dynamics w/ Substituted Constants:")
        disp(rd1_substituted)
        disp(rd2_substituted)
        disp(rd3_substituted)
        end

        tk1 = tk1_substituted;
        tk2 = tk2_substituted;
        tk3 = tk3_substituted;
        td1 = td1_substituted;
        td2 = td2_substituted;
        td3 = td3_substituted;
        rk1 = rk1_substituted;
        rk2 = rk2_substituted;
        rk3 = rk3_substituted;
        rd1 = rd1_substituted;
        rd2 = rd2_substituted;
        rd3 = rd3_substituted;
    end

end
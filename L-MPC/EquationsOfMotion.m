% Define the linearization function
function [tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3] = EquationsOfMotion(state, input, eom_params)
    % extract eom params
    constants = eom_params{1};
    current_index = eom_params{2};
    mass = eom_params{3};
    wind = eom_params{4};
    symbolic = eom_params{5};
    debug = eom_params{6};

    if isempty(constants) && symbolic==false
        error("Symbolic argument is set to false but the passed constants argument is empty!")
    end
    
    if (symbolic==true)
        % Create variables
        syms Ix Iy Iz Ax Ay Az kdx kdy kdz xdot_w ydot_w zdot_w l kf km ka m g
    else
        % K = [Ix, Iy, Iz, Ax, Ay, Az, kdx, kdy, kdz, xdot_w, ydot_w, zdot_w, l, kf, km, ka, m, g];
        Ix = constants(1);
        Iy = constants(2);
        Iz = constants(3);
        Ax = constants(4);
        Ay = constants(5);
        Az = constants(6);
        kdx = constants(7);
        kdy = constants(8);
        kdz = constants(9);
        xdot_w = constants(10);
        ydot_w = constants(11);
        zdot_w = constants(12);
        l = constants(13);
        kf = constants(14);
        km = constants(15);
        ka = constants(16);
        m = constants(17);
        g = constants(18);

        % [m, Ix, Iy, Iz] = get_mass(constants, current_time, mass_type);
        % extract mass constants
        m = mass(current_index, 1);
        Ix = mass(current_index, 2);
        Iy = mass(current_index, 3);
        Iz = mass(current_index, 4);
        
        % extract wind constants
        xdot_w = wind(current_index, 1);
        ydot_w = wind(current_index, 2);
        zdot_w = wind(current_index, 3);
    end

    x = state(1);
    y = state(2);
    z = state(3);
    u = state(4);
    v = state(5);
    w = state(6);
    phi = state(7);
    theta = state(8);
    psy = state(9);
    p = state(10);
    q = state(11);
    r = state(12);

    w1 = input(1);
    w2 = input(2);
    w3 = input(3);
    w4 = input(4);
    
    tk1_sym = u;
    tk2_sym = v;
    tk3_sym = w;

    td1_sym = (kf*(w1^2 + w2^2 + w3^2 + w4^2) * (sin(phi)*sin(psy)+cos(phi)*sin(theta)*cos(psy)) - (ka*Ax*((xdot_w-u)^2))) / m;
    td2_sym = (kf*(w1^2 + w2^2 + w3^2 + w4^2) * (-sin(phi)*cos(psy)+cos(phi)*sin(theta)*sin(psy)) - (ka*Ay*((ydot_w-v)^2))) / m;
    td3_sym = (kf*(w1^2 + w2^2 + w3^2 + w4^2) * cos(phi)*cos(theta) - m*g - (ka*Az*((zdot_w-w)^2))) / m;
    
    rk1_sym = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
    rk2_sym = q*cos(phi) - r*sin(phi);
    rk3_sym = q*sin(phi)*sec(theta) + r*cos(phi)*sec(theta);
    
    rd1_sym = (-((Iz-Iy)*q*r) + l*kf*(-w1^2 - w2^2 + w3^2 + w4^2))/Ix;
    rd2_sym = (-((Ix-Iz)*p*r) + l*kf*(-w1^2 + w2^2 + w3^2 - w4^2))/Iy;
    rd3_sym = (-((Iy-Ix)*p*q) + km*(w1^2 - w2^2 + w3^2 - w4^2))/Iz;
    
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

    elseif (isempty(constants)==false) && (symbolic==true)
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
    else
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
    end

end

function [m, Ix, Iy, Iz] = get_mass(constants, t, mass_type)
    mass_rate = 0.05;
    t1 = 12.5; % End time of continuous drop
    t0 = 7.5; % Start Time of continuous drop
    t_drop = 10; % Time of descrete drop
    Ix0 = constants(1);
    Iy0 = constants(2);
    Iz0 = constants(3);
    m0 = constants(17);

   switch mass_type
    case "constant"
       m = m0;
       Ix = Ix0;
       Iy = Iy0;
       Iz = Iz0;

    case "continuous"
       if t > t0
            m = m0 + (mass_rate*(t1-t0)) - mass_rate*(t-t0);
            if (m < m0)
                % minimal total mass of the drone is the mass of the drone with
                % no payload
                m = m0;
            end
            Ix = Ix0 + 0.00040757*(m-m0);
            Iy = Iy0 + 0.00040757*(m-m0);
            Iz = Iz0 + 0.00040757*(m-m0);
       else
           m = m0;
           Ix = Ix0;
           Iy = Iy0;
           Iz = Iz0;
       end

    case "discrete"
       delta_mass = m0*0.75;
       m = m0+ delta_mass -(1/(1+exp(-10000*(t-t_drop)))*delta_mass);
       Ix = Ix0 + 0.00040757*(m-m0);
       Iy = Iy0 + 0.00040757*(m-m0);
       Iz = Iz0 + 0.00040757*(m-m0);

    end
end


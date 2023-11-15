% quadrotor dynamics with an MRP for attitudebVal
syms w1 w2 w3 w4 phi theta psy p q r x y z
syms u v w

omega = [p, q, r]


Q = rot_from_euler(phi,theta,psy)

ang_vel_transform = [1 sin(phi)*tan(theta) cos(phi)*tan(theta)
                     0 cos(phi)            -sin(phi)
                     0 sin(phi)*sec(theta) cos(phi)*sec(theta)]
% constants = [Ix, Iy, Iz, Ax, Ay, Az, kdx, kdy, kdz, xdot_w, ydot_w, zdot_w, l, kf, km, ka, m, g];
mass=constants(17)
J = [constants(1), 0, 0; 0, constants(2), 0; 0, 0, constants(3) ]
gravity= 9.81
L= constants(13)
kf= constants(14)
km= constants(15)


F1 = kf*w1^2 % max(0,kf*w1)
F2 = kf*w2^2 % max(0,kf*w2)
F3 = kf*w3^2 % max(0,kf*w3)
F4 = kf*w4^2 % max(0,kf*w4)
F = [0., 0., F1+F2+F3+F4] %total rotor force in body frame

M1 = km*w1^2
M2 = km*w2^2
M3 = km*w3^2
M4 = km*w4^2
tau = [L*(-F1-F2+F3+F4), L*(-F1+F2+F3-F4), (M1-M2+M3-M4)] %total rotor torque in body frame

f_g = mass*gravity + Q*F' % forces in world frame

f = [
    u; v; w;
    f_g/mass
    ang_vel_transform * omega'
    inv(J)*(tau' - cross(omega', J*omega'))
]

% Define the state, control input, and output vectors
    state = [x; y; z; u; v; w; phi; theta; psy; p; q; r];
    

    
    control = [w1; w2; w3; w4];

% Determine the jacobians
    A = jacobian(f,state);
    B = jacobian(f,control);

% Create the matlab functions
    % Create QuadrotorStateFcn.m with current state and control vectors as inputs and the state time-derivative as outputs
    matlabFunction(f,"File","QuadrotorStateFcn", "Vars",{state,control});
    
    % Create QuadrotorStateJacobianFcn.m with current state and control vectors as inputs and the Jacobians of the state time-derivative as outputs
    matlabFunction(A,B,"File","QuadrotorStateJacobianFcn", "Vars",{state,control});

function rotm = rot_from_euler(phi,theta,psy)

    rotm =[
    (cos(psy)*cos(theta))   (cos(psy)*sin(theta)*sin(phi)-sin(psy)*cos(phi))   (sin(phi)*sin(psy)+cos(phi)*sin(theta)*cos(psy));
    (sin(psy)*cos(theta))   (sin(psy)*sin(theta)*sin(phi)+cos(psy)*cos(phi))   (-sin(phi)*cos(psy)+cos(phi)*sin(theta)*sin(psy));
    (-sin(theta))           (cos(theta)*sin(phi))                              (cos(phi)*cos(theta));
    ]
end
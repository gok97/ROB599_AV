%% Script to generate the state and jacobian functions starting from the equations of motion

% Prepare the equations of motion
    % Create variables
    syms t Ix Iy Iz Ax Ay Az kdx kdy kdz xdot_w ydot_w zdot_w l kf km ka m g
    
    % Define the states
    syms x(t) y(t) z(t) phi(t) theta(t) psy(t) p(t) q(t) r(t)
    
    % Define the state derivatives
    xdot = diff(x, t);
    ydot = diff(y, t);
    zdot = diff(z, t);

    % Define the inputs
    syms w1 w2 w3 w4
    
    % Define the equations
    tk1_sym = xdot;
    tk2_sym = ydot;
    tk3_sym = zdot;
    
    td1_sym = ((kf*(w1^2 + w2^2 + w3^2 + w4^2))*(sin(phi(t))*sin(psy(t))+cos(phi(t))*sin(theta(t))*cos(psy(t))) - (ka*Ax*((xdot_w-xdot)^2)))/m;
    td2_sym = ((kf*(w1^2 + w2^2 + w3^2 + w4^2))*(-sin(phi(t))*cos(psy(t))+sin(phi(t))*sin(theta(t))*cos(psy(t)))  - (ka*Ay*((ydot_w-ydot)^2)))/m;
    td3_sym = (-m*g + (kf*(w1^2 + w2^2 + w3^2 + w4^2))*(cos(phi(t))*cos(theta(t)))  - (ka*Az*((zdot_w-zdot)^2)))/m;
    
    rk1_sym = p + q*sin(phi(t))*tan(theta(t)) + r*cos(phi(t))*tan(theta(t));
    rk2_sym = q*cos(phi(t)) - r*sin(phi(t));
    rk3_sym = q*sin(phi(t))*sec(theta(t)) + r*cos(phi(t))*sec(theta(t));
    
    rd1_sym = (-((Iz-Iy)*q*r) - q*(w1+w2+w3+w4) - kdx*p + l*kf*(-w1^2 - w2^2 + w3^2 + w4^2))/Ix;
    rd2_sym = (-((Ix-Iz)*p*r) + p*(w1+w2+w3+w4) - kdy*q + l*kf*(-w1^2 + w2^2 + w3^2 - w4^2))/Iy;
    rd3_sym = (-((Iy-Ix)*p*q) - kdz*r + km*(w1^2 - w2^2 + w3^2 - w4^2))/Iz;


% Substitute the constant
    tk1 = (subs(tk1_sym, [Ix; Iy; Iz; Ax; Ay; Az; kdx; kdy; kdz; xdot_w; ydot_w; zdot_w; l; kf; km; ka; m; g], constants));
    tk2 = (subs(tk2_sym, [Ix; Iy; Iz; Ax; Ay; Az; kdx; kdy; kdz; xdot_w; ydot_w; zdot_w; l; kf; km; ka; m; g], constants));
    tk3 = (subs(tk3_sym, [Ix; Iy; Iz; Ax; Ay; Az; kdx; kdy; kdz; xdot_w; ydot_w; zdot_w; l; kf; km; ka; m; g], constants));
    td1 = (subs(td1_sym, [Ix; Iy; Iz; Ax; Ay; Az; kdx; kdy; kdz; xdot_w; ydot_w; zdot_w; l; kf; km; ka; m; g], constants));
    td2 = (subs(td2_sym, [Ix; Iy; Iz; Ax; Ay; Az; kdx; kdy; kdz; xdot_w; ydot_w; zdot_w; l; kf; km; ka; m; g], constants));
    td3 = (subs(td3_sym, [Ix; Iy; Iz; Ax; Ay; Az; kdx; kdy; kdz; xdot_w; ydot_w; zdot_w; l; kf; km; ka; m; g], constants));
    rk1 = (subs(rk1_sym, [Ix; Iy; Iz; Ax; Ay; Az; kdx; kdy; kdz; xdot_w; ydot_w; zdot_w; l; kf; km; ka; m; g], constants));
    rk2 = (subs(rk2_sym, [Ix; Iy; Iz; Ax; Ay; Az; kdx; kdy; kdz; xdot_w; ydot_w; zdot_w; l; kf; km; ka; m; g], constants));
    rk3 = (subs(rk3_sym, [Ix; Iy; Iz; Ax; Ay; Az; kdx; kdy; kdz; xdot_w; ydot_w; zdot_w; l; kf; km; ka; m; g], constants));
    rd1 = (subs(rd1_sym, [Ix; Iy; Iz; Ax; Ay; Az; kdx; kdy; kdz; xdot_w; ydot_w; zdot_w; l; kf; km; ka; m; g], constants));
    rd2 = (subs(rd2_sym, [Ix; Iy; Iz; Ax; Ay; Az; kdx; kdy; kdz; xdot_w; ydot_w; zdot_w; l; kf; km; ka; m; g], constants));
    rd3 = (subs(rd3_sym, [Ix; Iy; Iz; Ax; Ay; Az; kdx; kdy; kdz; xdot_w; ydot_w; zdot_w; l; kf; km; ka; m; g], constants));

% Define the state, control input, and output vectors
    state = [x(t); y(t); z(t); xdot; ydot; zdot; phi(t); theta(t); psy(t); p(t); q(t); r(t)];
    state = subsStateVars(state, t);
    
    f = [tk1; tk2; tk3; td1; td2; td3; rk1; rk2; rk3; rd1; rd2; rd3];
    f = subsStateVars(f,t);
    
    control = [w1; w2; w3; w4];

% Determine the jacobians
    A = jacobian(f,state);
    B = jacobian(f,control);

% Create the matlab functions
    % Create QuadrotorStateFcn.m with current state and control vectors as inputs and the state time-derivative as outputs
    matlabFunction(f,"File","QuadrotorStateFcn", "Vars",{state,control});
    
    % Create QuadrotorStateJacobianFcn.m with current state and control vectors as inputs and the Jacobians of the state time-derivative as outputs
    matlabFunction(A,B,"File","QuadrotorStateJacobianFcn", "Vars",{state,control});


%% Helper Functions
function stateExpr = subsStateVars(timeExpr,var)
    if nargin == 1 
        var = sym("t");
    end
    repDiff = @(ex) subsStateVarsDiff(ex,var);
    stateExpr = mapSymType(timeExpr,"diff",repDiff);
    repFun = @(ex) subsStateVarsFun(ex,var);
    stateExpr = mapSymType(stateExpr,"symfunOf",var,repFun);
    stateExpr = formula(stateExpr);
end

function newVar = subsStateVarsFun(funExpr,var)
    name = symFunType(funExpr);
    name = replace(name,"_Var","");
    stateVar = "_" + char(var);
    newVar = sym(name + stateVar);
end

function newVar = subsStateVarsDiff(diffExpr,var)
    if nargin == 1 
      var = sym("t");
    end
    c = children(diffExpr);
    if ~isSymType(c{1},"symfunOf",var)
      % not f(t)
      newVar = diffExpr;
      return;
    end
    if ~any([c{2:end}] == var)
      % not derivative wrt t only
      newVar = diffExpr;
      return;
    end
    name = symFunType(c{1});
    name = replace(name,"_Var","");
    extension = "_" + join(repelem("d",numel(c)-1),"") + "ot";
    stateVar = "_" + char(var);
    newVar = sym(name + extension + stateVar);
end
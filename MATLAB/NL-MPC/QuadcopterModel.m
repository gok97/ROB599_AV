%% Script to gModelenerate the state and jacobian functions startingModel from the equations of motion
% Extract constants to make the function more legible
IxModel = modelConstants(1);
IyModel = modelConstants(2);
IzModel = modelConstants(3);
AxModel = modelConstants(4);
AyModel = modelConstants(5);
AzModel = modelConstants(6);
kdxModel = modelConstants(7);
kdyModel = modelConstants(8);
kdzModel = modelConstants(9);
xdotWModel = modelConstants(10);
ydotWModel = modelConstants(11);
zdotWModel = modelConstants(12);
lModel = modelConstants(13);
kfModel = modelConstants(14);
kmModel = modelConstants(15);
kaModel = modelConstants(16);
mModel = modelConstants(17);
gModel = modelConstants(18);

% Define the states
syms t
syms x(t) y(t) z(t) phi(t) theta(t) psy(t) p(t) q(t) r(t)

% Define the state derivatives
xdot = diff(x, t);
ydot = diff(y, t);
zdot = diff(z, t);

% Define the inputs
syms w1 w2 w3 w4

% Define the equations of motion
tk1Sym = xdot;
tk2Sym = ydot;
tk3Sym = zdot;

td1Sym = ((kfModel*(w1^2 + w2^2 + w3^2 + w4^2))*(sin(phi(t))*sin(psy(t))+cos(phi(t))*sin(theta(t))*cos(psy(t))) - (kaModel*AxModel*((xdotWModel-xdot)^2)))/mModel;
td2Sym = ((kfModel*(w1^2 + w2^2 + w3^2 + w4^2))*(-sin(phi(t))*cos(psy(t))+sin(psy(t))*sin(theta(t))*cos(phi(t)))  - (kaModel*AyModel*((ydotWModel-ydot)^2)))/mModel;
td3Sym = (-mModel*gModel + (kfModel*(w1^2 + w2^2 + w3^2 + w4^2))*(cos(phi(t))*cos(theta(t)))  - (kaModel*AzModel*((zdotWModel-zdot)^2)))/mModel;

rk1Sym = p + q*sin(phi(t))*tan(theta(t)) + r*cos(phi(t))*tan(theta(t));
rk2Sym = q*cos(phi(t)) - r*sin(phi(t));
rk3Sym = q*sin(phi(t))*sec(theta(t)) + r*cos(phi(t))*sec(theta(t));

rd1Sym = (-((IzModel-IyModel)*q*r)  - kdxModel*p + lModel*kfModel*(-w1^2 - w2^2 + w3^2 + w4^2))/IxModel;
rd2Sym = (-((IxModel-IzModel)*p*r)  - kdyModel*q + lModel*kfModel*(-w1^2 + w2^2 + w3^2 - w4^2))/IyModel;
rd3Sym = (-((IyModel-IxModel)*p*q) - kdzModel*r + kmModel*(w1^2 - w2^2 + w3^2 - w4^2))/IzModel;

% Define the state, control input, and output vectors
state = [x(t); y(t); z(t); xdot; ydot; zdot; phi(t); theta(t); psy(t); p(t); q(t); r(t)];
state = subsStateVars(state, t);

f = [tk1Sym; tk2Sym; tk3Sym; td1Sym; td2Sym; td3Sym; rk1Sym; rk2Sym; rk3Sym; rd1Sym; rd2Sym; rd3Sym];
f = subsStateVars(f,t);

control = [w1; w2; w3; w4];

% Determine the jacobians
A = jacobian(f,state);
B = jacobian(f,control);

% Create QuadrotorStateFcn.m with current state and control vectors as inputs and the state time-derivative as outputs
matlabFunction(f,"File", 'QuadrotorStateFcn', "Vars",{state,control});

% Create QuadrotorStateJacobianFcn.m with current state and control vectors as inputs and the Jacobians of the state time-derivative as outputs
matlabFunction(A,B,"File",'QuadrotorStateJacobianFcn', "Vars",{state,control});


%% Helper Functions [FROM MATHWORKS]
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
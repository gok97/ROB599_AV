%% Prepare Environment
clear
clc

%% Debug code
syms x y
r = sqrt(x^2 + y^2);
ht = matlabFunction(r, 'Vars', [y, x])

ht(3, 4)
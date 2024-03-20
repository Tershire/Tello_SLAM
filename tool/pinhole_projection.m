% pinhole_projection.m

%///////////////////////%
%/ IONLAB ISAE-SUPAERO /%
%/ TELLO SLAM PROJECT  /%
%////////////////////////

% 2024 MAR 20
% Wonhee LEE

% reference:

clear
close all

%% projection model
syms p_u [2, 1]
syms P_c [3, 1]

syms cx cy fx fy

p_u(1) = fx*P_c(1)/P_c(3) + cx
p_u(2) = fy*P_c(2)/P_c(3) + cy

%% linearize
J_Pc = jacobian(p_u, P_c);
H = simplify(J_Pc)

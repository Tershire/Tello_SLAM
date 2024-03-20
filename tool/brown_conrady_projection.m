% brown_conrady_projection.m

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
syms p_d [2, 1]
syms p_u_RHS [2, 1]
syms P_c [3, 1]

syms cx cy fx fy
syms k1 k2 p1 p2

p_u_LHS(1) = fx*P_c(1)/P_c(3) + cx
p_u_LHS(2) = fy*P_c(2)/P_c(3) + cy

% r = norm(p_d)
r = sqrt((p_d(1))^2 + (p_d(2))^2)

p_u_RHS(1) = p_d(1) + p_d(1)*(k1*r^2 + k2*r^4) + (p1*(r^2 + 2*(p_d(1))^2) + 2*p2*p_d(1)*p_d(2))
p_u_RHS(2) = p_d(2) + p_d(2)*(k1*r^2 + k2*r^4) + (2*p1*p_d(1)*p_d(2) + p2*(r^2 + 2*(p_d(2))^2))

%% linearize
J_Pc = jacobian(p_u_LHS, P_c);
J_Pc = simplify(J_Pc)

J_pd = jacobian(p_u_RHS, p_d);
J_pd = simplify(J_pd)

%%
H = inv(J_pd)*J_Pc
H = simplify(H)

clc; clear; close all;

A = load('out_obs.txt');
P = load('out_params_est_w_analytic_jacobian.txt');
%P = [5.112966	0.098694	0.950725];

figure;
plot(A(:,1), A(:,2),'.-');
x = A(:,1);
% x[i]=p[0]*exp(-p[1]*i) + p[2];
yest = P(1) * exp(-P(2) .* x) + P(3);

hold on;
plot(x, yest, 'r-');
legend('Observation' ,'Estimated');
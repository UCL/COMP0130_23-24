function [Fd, Qd] = continuousToDiscrete(Fc, Qc, dt)

% Use Van Loan's algorithm to work out a discrete => continuous
% transformation

n = size(Fc, 1);

bigA = zeros(2*n);

bigA(1:n,1:n) = -Fc * dt;
bigA(1:n, n+1:end) = Qc * dt;
bigA(n+1:end, n+1:end) = Fc' * dt;

bigB=expm(bigA);

Fd = bigB(n+1:end, n+1:end)';
Qd = Fd * bigB(1:n, n+1:end);

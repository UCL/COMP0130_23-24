% Script to tune / test the process model

% Specify some details

alpha = 0.1;
beta = 0.05;
sigmaQ = 1;

Fc=[0 1 0 0;-alpha -beta 0 0;0 0 0 1;0 0 -alpha -beta];

Qc = [0 0 0 0;0 1 0 0;0 0 0 0;0 0 0 1] * sigmaQ;

[Fd, Qd] = continuousToDiscrete(Fc, Qc, 0.1);

steps = 20000;

xStore = zeros(4, steps);
PStore = zeros(4, steps);
P = zeros(4);

for k = 2 : steps
    xStore(:, k) = Fd * xStore(:, k - 1) + sqrtm(Qd) * randn(4, 1);
    P = Fd * P * Fd' + Qd;
    PStore(:, k) = diag(P);
end

plot(PStore(1,:))
% This script runs the sampler to generate plots which illustrate the
% uncertainty.
%
% It uses something called Riemannian Hamiltonian Monte Carlo sampling.
% The algorithm falls outside the scope of this module. The main thing to
% note is that it (a) requires manual fiddling of parameters and (b) can be
% very slow!

% It needs to be run after the dynamic example scripts.

% RHMC is parameterised by L (the number of steps) and epsilon (the size of
% each step). These are not chosen adaptively.

function sampleGraph(graph, numSamples, L, epsilon, videoFileName)

% Check it's the correct type
assert(isa(graph, 'g2o.core.SparseOptimizer') == true)

% Create the sampler
sampler = g2o.sampling.RiemannianHamiltonianSampler(graph);
%sampler = g2o.sampling.HamiltonianSampler(graph);

% Handle defaults
if (nargin < 5)
    videoWriter = [];
else
    videoWriter = VideoWriter(videoFileName, 'Motion JPEG AVI');
    open(videoWriter);
end

if (nargin < 4)
    epsilon = 0.0012;
end

if (nargin < 3)
    L = 20;
end

if (nargin < 2)
    numSamples = 1000;
end

% Set the number of steps and step length
sampler.setParameters(L, epsilon);

% Get the initial estimate
theta0 = graph.computeMarginals();

% Various book keeping parameters
theta = theta0;
numRefresh = 0;
thetaStore = NaN(length(theta0), numSamples);
numAccepted = 0;

for k = 1 : numSamples
    if (rand < 0.01)
        theta = theta0;
        numRefresh = numRefresh + 1;
    end
    [theta, accepted] = sampler.sample([], theta);
    thetaStore(:, k) = theta;
    if (accepted == true)
        figure(3)
        plot(thetaStore(1:4:end,1:k), thetaStore(3:4:end,1:k))
        drawnow
        numAccepted = numAccepted + 1;
        if (isempty(videoWriter) == false)
            writeVideo(videoWriter, getframe(gcf));
        end
    end
    fprintf('k=%03d;numAccepted=%03d;numRefresh=%03d\n', k, numAccepted, numRefresh)
end

if (isempty(videoWriter) == false)
    close(videoWriter)
end

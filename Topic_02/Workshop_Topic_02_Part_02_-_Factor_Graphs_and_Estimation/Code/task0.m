% Simple static object example. This shows how to build a simple graph to
% estimate the state of a single object from a bunch of measurements of
% its position. This is a very simple version of using a GPS system to
% estimate the position of a static object.

import g2o.core.*;
import one_d_static.*;

% Some parameters
numberOfMeasurements = 1;
sigmaR = 10;

% The information (inverse covariance) for the measurement edge
omegaR = 1/sigmaR;

% Ground truth location
trueX = 10;

% Sample the noises for the different observations
z = trueX + sqrt(sigmaR) * randn(numberOfMeasurements, 1);

% Create the graph
graph = SparseOptimizer();
algorithm = GaussNewtonOptimizationAlgorithm();
graph.setAlgorithm(algorithm);

% Create the vertex. This contains the state we want to estimate.
v = ObjectStateVertex();

% Set the initial estimate. We have to set some initial value. This ideally
% shouldn't be too far from the final soluton. Here we use the first
% measurement.
v.setEstimate(z(1));

% Added the vertex to the graph. The graph now knows that we have some
% states we want to estimate.
graph.addVertex(v);

% Create a single edge. Each edge corresponds to a position measurement.

% Create the measurement edge
e = ObjectMeasurementEdge();

% Link it so that it connects to the vertex we want to estimate
e.setVertex(1, v);

% Set the measurement value and the measurement covariance
e.setMeasurement(z(1));
e.setInformation(omegaR);

% Add the edge to the graph; the graph now knows we have these edges
% which need to be added
graph.addEdge(e);

% Graph construction complete

% Initialise the optimization. If you ever change the structure of the
% graph (e.g., add remove edges and vertices) you must call this before
% calling optimize. If you don't change the graph structure, you can call
% optimze multiple times without calling initializeOptimization().
graph.initializeOptimization();

% Run the optimizer. By default it does at most 10 iterations, but you can
% pass the number in optionally.
graph.optimize();

% Get the estimate and covariance. In graph-land, getting the covariance
% matrix is described as "computing the marginals". Note the vector X is
% the "big state vector" which consists of the states from all the vertices
% concatenated on one another. Similarly, PX is a sparse covariance matrix
% which contains the covariances of the vertices.
[X, PX] = graph.computeMarginals();

% To extract the state from a vertex, do
% idx = vertex.hessianIndex();
%
% The state estimate is X(idx)
%
% The covariance of this state estimate is P(idx, idx)
%
% This looks a bit awkward, but it's the convention for using this graph
% library.

idx = v.hessianIndex();

disp('Estimated target position')
xEst = X(idx)

disp('Estimated target covariance:')
PEst = PX(idx, idx)



function Z = hFunRB(x, landmarks, maxRange, maxBeta, R)

% Observation vector
Z.z = zeros(2, 0);
Z.landmarkIDs = [];

% LandmarkIDs are just the indices for the landmark
landmarkIDs = 1:size(landmarks, 2);

% Distance to the landmarks
dx = landmarks - repmat(x(1:2), 1, size(landmarks, 2));

% Find the landmarks which are close enough to be seen
r = sqrt(sum(dx.^2, 1));
idx = find(r < maxRange);

% Nothing to do if empty
if (isempty(idx))
    return
end

% Reduce the search to the potentially visible beacons
r = r(idx);
landmarks = landmarks(:, idx);
landmarkIDs = landmarkIDs(idx);
dx = dx(:, idx);

% Work out the bearings to the landmarks
beta = g2o.stuff.normalize_theta(atan2(dx(2,:), dx(1,:)) - x(3));

idx = find(abs(beta) < maxBeta);

% Nothing to do if empty
if (isempty(idx))
    return
end

% Now strip off the features outside the view cone of the sensor
r = r(idx);
beta = beta(idx);
Z.landmarkIDs = landmarkIDs(idx);

% Construct the observations
Z.z = [r;beta] + sqrtm(R) * randn(2, length(idx));



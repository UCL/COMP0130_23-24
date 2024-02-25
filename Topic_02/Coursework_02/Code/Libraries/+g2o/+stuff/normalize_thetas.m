function theta = normalize_thetas(theta)

%return

idx = find(theta < -pi);

theta(idx) = theta(idx) + 2 * pi;

idx = find(theta > pi);

theta(idx) = theta(idx) - 2 * pi;

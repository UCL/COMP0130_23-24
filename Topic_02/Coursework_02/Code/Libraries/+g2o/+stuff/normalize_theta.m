function theta = normalize_theta(theta)

%return

if (theta < -pi)
    theta = theta + 2* pi;
elseif (theta > pi)
    theta = theta - 2* pi;
end
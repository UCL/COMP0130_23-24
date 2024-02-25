% Some tests

pose1 = gtsam.Pose2(0, 0, -pi/2);
rpose12 = gtsam.Pose2(1, 0, 0);
rpose23 = gtsam.Pose2(1, 0, pi/2);

pose2 = pose1.compose(rpose12)
pose3 = pose2.compose(rpose23)

function key_points = KeyPointStraightPath()

    pt1.x = 0.0; pt1.y = 0.0; pt1.s = 0.0; 
    pt1.theta = 0; pt1.kappa = 0; pt1.dkappa = 0.0;

    pt2.x = 0.0; pt2.y = 0.0; pt2.s = 100; 
    pt2.theta = 0; pt2.kappa = 0; pt2.dkappa = 0.0;

    key_points = [pt1, pt2];
end
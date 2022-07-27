function key_points = KeyPointLeftUTurnPath()
    straight_length = 30.0;
    radius = 30;
    circumference = 2.0 * pi * radius;
    
    pt1.x = 0.0; pt1.y = 0.0; pt1.s = 0.0; 
    pt1.theta = 0; pt1.kappa = 0.0; pt1.dkappa = 0.0;

    pt2.x = 0.0; pt2.y = 0.0; pt2.s = straight_length; 
    pt2.theta = 0.0; pt2.kappa = 0.0; pt2.dkappa = 0.0;
    
    pt3.x = 0.0; pt3.y = 0.0; pt3.s = circumference / 4 * 2 + straight_length; 
    pt3.theta = 0.5 * pi; pt3.kappa = 1.0 / radius; pt3.dkappa = 0.0;
    
    pt4.x = 0.0; pt4.y = 0.0; pt4.s = circumference / 4 * 2 + straight_length + circumference / 4 * 2; 
    pt4.theta = 1.0 * pi; pt4.kappa = 0.0 / radius; pt4.dkappa = 0.0;
    
    pt5.x = 0.0; pt5.y = 0.0; pt5.s = circumference / 4 * 2 + straight_length + circumference / 4 * 2 + straight_length; 
    pt5.theta = 1.0 * pi; pt5.kappa = 0.0; pt5.dkappa = 0.0;
    
    key_points = [pt1, pt2, pt3, pt4, pt5];
end
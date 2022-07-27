function key_points = KeyPointSTurnPath()
    straight_length = 50.0;
    radius_1 = 20;
    circumference_1 = 2.0 * pi * radius_1;
    
    radius_2 = 40;
    circumference_2 = 2.0 * pi * radius_2;
    
    radius_3 = 100;
    circumference_3 = 2.0 * pi * radius_3;
    
    radius_4 = 30;
    circumference_4 = 2.0 * pi * radius_4;

    pt1.x = 0.0; pt1.y = 0.0; pt1.s = 0.0; 
    pt1.theta = 0; pt1.kappa = 0.0; pt1.dkappa = 0.0;

    pt2.x = 0.0; pt2.y = 0.0; pt2.s = straight_length; 
    pt2.theta = 0.0; pt2.kappa = 0.0; pt2.dkappa = 0.0;

    pt3.x = 0.0; pt3.y = 0.0; pt3.s = circumference_1 / 4 + straight_length; 
    pt3.theta = 0.5 * pi; pt3.kappa = 1.0 / radius_1; pt3.dkappa = 0.0;

    pt4.x = 0.0; pt4.y = 0.0; pt4.s = circumference_1 / 4 * 2 + straight_length; 
    pt4.theta = 1.0 * pi; pt4.kappa = 1.0 / radius_1; pt4.dkappa = 0.0;

    pt5.x = 0.0; pt5.y = 0.0; pt5.s = circumference_1 / 4 * 2 + 2 * straight_length; 
    pt5.theta = 1.0 * pi; pt5.kappa = 0.0; pt5.dkappa = 0.0;
    
    pt5.x = 0.0; pt5.y = 0.0; pt5.s = circumference_1 / 4 * 2 + 2 * straight_length; 
    pt5.theta = 1.0 * pi; pt5.kappa = 0.0; pt5.dkappa = 0.0;
    
    pt6.x = 0.0; pt6.y = 0.0; pt6.s = circumference_1 / 4 * 2 + 2 * straight_length + circumference_2 / 4; 
    pt6.theta = 0.5 * pi; pt6.kappa = 1.0 / radius_2; pt6.dkappa = 0.0;
    
    pt7.x = 0.0; pt7.y = 0.0; pt7.s = circumference_1 / 4 * 2 + 2 * straight_length + circumference_2 / 4 + circumference_3 / 4; 
    pt7.theta = 0.0 * pi; pt7.kappa = 1.0 / radius_3; pt7.dkappa = 0.0;
    
    pt8.x = 0.0; pt8.y = 0.0; pt8.s = circumference_1 / 4 * 2 + 2 * straight_length + circumference_2 / 4 + circumference_3 / 4 + 10.0; 
    pt8.theta = 0.0 * pi; pt8.kappa = 0.0; pt8.dkappa = 0.0;
    
    pt9.x = 0.0; pt9.y = 0.0; pt9.s = circumference_1 / 4 * 2 + 2 * straight_length + circumference_2 / 4 + circumference_3 / 4 + 10.0 + circumference_4 / 4 * 2; 
    pt9.theta = 1.0 * pi; pt9.kappa = -1.0 / radius_4; pt9.dkappa = 0.0;
    
    pt10.x = 0.0; pt10.y = 0.0; pt10.s = circumference_1 / 4 * 2 + 2 * straight_length + circumference_2 / 4 + circumference_3 / 4 + 10.0 + circumference_4 / 4 * 2 + circumference_4 / 4 * 3; 
    pt10.theta = 0.0 * pi; pt10.kappa = 1.0 / radius_4; pt10.dkappa = 0.0;
    
    pt11.x = 0.0; pt11.y = 0.0; pt11.s = circumference_1 / 4 * 2 + 2 * straight_length + circumference_2 / 4 + circumference_3 / 4 + 10.0 + circumference_4 / 4 * 2 + circumference_4 / 4 * 3 + 10.0; 
    pt11.theta = 0.0 * pi; pt11.kappa = 0.0; pt11.dkappa = 0.0;

    key_points = [pt1, pt2, pt3, pt4, pt5, pt6, pt7, pt8, pt9, pt10, pt11];
end
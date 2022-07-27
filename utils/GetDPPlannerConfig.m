function config = GetDPPlannerConfig()
config.nfe = 320;
config.tf = 16;
config.nominal_velocity = 10.0;
config.weight_obstacle = 1e10;
config.weight_lateral = 1;
config.weight_lateral_change = 0.5;
config.weight_lateral_velocity_change = 1.0;
config.weight_longitudinal_velocity_bias = 100.0;
config.weight_longitudinal_velocity_change = 1.0;

config.max_l_prime = 5.0;
config.max_l_dot = 5.0;
config.max_longitudinal_velocity_change = 10.0;

end
function config = GetLIOCPPlannerConfig()
config.weight_u = 1;
config.weight_x = 5;
config.weight_y = 5;
config.weight_theta = 5;
config.weight_omega = 1000.0;
config.max_iter = 5;
config.weight_penalty_init = 1e5;
config.alpha = 10;
config.varepsilon_tol = 1e-4;
end
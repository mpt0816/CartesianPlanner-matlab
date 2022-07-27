function params = GetVehicleParams()
params.wheel_base = 2.998;
params.front_oh = 0.941;
params.rear_oh = 0.941;
params.width = 1.896;
params.length = params.wheel_base + params.front_oh + params.rear_oh;
params.chassis_height = 0.4;
params.radius = hypot(params.length / 4.0, params.width / 2.0);
params.rear_to_center = 0.5 * params.length - params.rear_oh;
params.collision_buffer = 0.1;

params.max_velocity = 36.0;
params.max_acceleration = 5.0;
params.max_jerk = 10.0;
params.max_delta = 0.85;
params.max_omega = 1.5;
end
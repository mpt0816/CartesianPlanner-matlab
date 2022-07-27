classdef SafeCorridor < handle
    
    properties
        env_
        dp_config_
        vehicle_param_
        max_iter_ = 500
        incremental_limit_ = 5.0;
    end
    
    methods
        function obj = SafeCorridor(env)
            obj.env_ = env;
            obj.dp_config_ = GetDPPlannerConfig();
            obj.vehicle_param_ = GetVehicleParams();
        end
        
        function [flag, constraints] = FormulateConstraints(obj, traj)
            radius = obj.vehicle_param_.radius;
            constraints.f_min_x = zeros(obj.dp_config_.nfe, 1);
            constraints.f_max_x = zeros(obj.dp_config_.nfe, 1);
            constraints.f_min_y = zeros(obj.dp_config_.nfe, 1);
            constraints.f_max_y = zeros(obj.dp_config_.nfe, 1);
            constraints.r_min_x = zeros(obj.dp_config_.nfe, 1);
            constraints.r_max_x = zeros(obj.dp_config_.nfe, 1);
            constraints.r_min_y = zeros(obj.dp_config_.nfe, 1);
            constraints.r_max_y = zeros(obj.dp_config_.nfe, 1);
            
            dt = obj.dp_config_.tf / (obj.dp_config_.nfe - 1);
            for i = 1 : 1 : obj.dp_config_.nfe
                time = i * dt;
                [front_x, front_y, rear_x, rear_y] = Utils.GetVehicleDiscPositions(traj(i), traj(obj.dp_config_.nfe + i), traj(2 * obj.dp_config_.nfe + i), obj.vehicle_param_);
                [flag, constraints.f_min_x(i, 1), ...
                       constraints.f_max_x(i, 1), ...
                       constraints.f_min_y(i, 1), ...
                       constraints.f_max_y(i, 1)] = obj.GenerateBox(time, front_x, front_y, radius);
                if ~flag
                    return; 
                end
                
                [flag, constraints.r_min_x(i, 1), ...
                       constraints.r_max_x(i, 1), ...
                       constraints.r_min_y(i, 1), ...
                       constraints.r_max_y(i, 1)] = obj.GenerateBox(time, rear_x, rear_y, radius);
                if ~flag
                    return; 
                end
            end
        end
        
        function [flag, min_x, max_x, min_y, max_y] = GenerateBox(obj, time, x, y, radius)
            real_x = x;
            real_y = y;
            inc = 4;
            while (obj.CheckCollision(time, real_x, real_y, 2 * radius, 2 * radius) && inc < obj.max_iter_ + 4)
                iter = fix(inc / 4);
                edge = rem(inc, 4);
                if edge == 0
                    real_x = x - iter * 0.05;
                elseif edge == 1
                    real_x = x + iter * 0.05;
                elseif edge == 2
                    real_y = y - iter * 0.05;
                elseif edge == 3
                    real_y = y + iter * 0.05;
                end
                inc = inc + 1;
            end
            
            if inc > obj.max_iter_
                flag = false;
                return;
            end
            
            inc = 4;
            %% Counterclockwise, start from +x direction
            blocked = [false, false, false, false];
            incremental = [0.0, 0.0, 0.0, 0.0];
            step = radius / 5;
            while (~all(blocked) && inc < obj.max_iter_ + 4)
                iter = fix(inc / 4);
                edge = rem(inc, 4) + 1;
                inc = inc + 1;
                
                if blocked(edge)
                    continue;
                end
                
                incremental(edge) = iter * step;
                inc_x = real_x + (incremental(1) - incremental(3)) / 2.0;
                inc_y = real_y + (incremental(2) - incremental(4)) / 2.0;
                inc_length = 2 * radius + incremental(1) + incremental(3);
                inc_width = 2 * radius + incremental(2) + incremental(4);
                if (obj.CheckCollision(time, inc_x, inc_y, inc_length, inc_width) || incremental(edge) >= obj.incremental_limit_)
                    incremental(edge) = incremental(edge) - step;
                    blocked(edge) = true;
                end
            end
            
            flag = true;
            max_x = real_x + incremental(1);
            min_x = real_x - incremental(3);
            max_y = real_y + incremental(2);
            min_y = real_y - incremental(4);
        end
        
        function out = CheckCollision(obj, time, x, y, length, width)
            box = Box2d(x, y, 0.0, length, width);
            out = obj.env_.CheckCollision(time, box);
        end
    end
end


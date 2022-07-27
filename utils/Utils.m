classdef Utils < handle
    
    methods
        function obj = Utils()
        end
    end
    
    methods(Static)
        function point = CalculatePointByHeading(pt, length)
            point = pt;
            point.x = pt.x - length * cos(pt.theta);
            point.y = pt.y - length * sin(pt.theta);
            point.theta = pt.theta;
        end
        
        function pts = CalculateConersFromRearPoint(pt_rear)
            params = GetVehicleParams();
            wheel_base = params.wheel_base;
            front_oh = params.front_oh;
            rear_oh = params.rear_oh;
            width = 0.5 * params.width;
            
            x = pt_rear.x; y = pt_rear.y; theta = pt_rear.theta;
            pt1.x = x + (front_oh + wheel_base) * cos(theta) - width * sin(theta);
            pt1.y = y + (front_oh + wheel_base) * sin(theta) + width * cos(theta);
            
            pt2.x = x - rear_oh * cos(theta) - width * sin(theta);
            pt2.y = y - rear_oh * sin(theta) + width * cos(theta);
            
            pt3.x = x - rear_oh * cos(theta) + width * sin(theta);
            pt3.y = y - rear_oh * sin(theta) - width * cos(theta);
            
            pt4.x = x + (front_oh + wheel_base) * cos(theta) + width * sin(theta);
            pt4.y = y + (front_oh + wheel_base) * sin(theta) - width * cos(theta);
            
            pts = [pt1, pt2, pt3, pt4];
        end
        
        function [front_x, front_y, rear_x, rear_y] = GetVehicleDiscPositions(x, y, heading, config)
            front_length = 0.75 * config.length - config.rear_oh;
            rear_length = 0.25 * config.length - config.rear_oh;
            
            front_x = x + front_length * cos(heading);
            front_y = y + front_length * sin(heading);
            rear_x = x + rear_length * cos(heading);
            rear_y = y + rear_length * sin(heading);
        end
        
        function [center_x, center_y] = GetVehicleCenterPosition(x, y, heading, config)
            center_x = x + config.rear_to_center * cos(heading);
            center_y = y + config.rear_to_center * sin(heading);
        end
        
        function guess = CalculateInitialGuess(coarse, start_state)
            dp_config = GetDPPlannerConfig();
            vehicle_param = GetVehicleParams();
            n = dp_config.nfe;
            tf = dp_config.tf;
            guess = zeros(12 * n, 1);
            %% updata x, y theta
            guess(1: n, 1) = coarse.x;
            guess(n + 1: 2 * n, 1) = coarse.y;
            guess(2 * n + 1: 3 * n, 1) = coarse.theta;
            
            %% update start point
            guess(1, 1) = start_state.x;
            guess(n + 1, 1) = start_state.y;
            guess(2 * n + 1, 1) = start_state.theta;
            guess(3 * n + 1, 1) = start_state.v;
            guess(4 * n + 1, 1) = start_state.delta;
            guess(5 * n + 1, 1) = start_state.a;
            guess(6 * n + 1, 1) = start_state.omega;
            guess(7 * n + 1, 1) = start_state.jerk;
            
            %% updata velocity, delta
            dt = tf / (n - 1);
            guess(3 * n + 2 : 4 * n, 1) = ...
                min(vehicle_param.max_velocity , ...
                hypot(guess(2 : n, 1) - guess(1 : n - 1, 1), ...
                guess(n + 2 : 2 * n, 1) - guess(n + 1 : 2 * n - 1, 1)) / dt);
            guess(4 * n + 2 : 5 * n, 1) = ...
                min(vehicle_param.max_delta, max(-vehicle_param.max_delta, ...
                atan((guess(2 * n + 2 : 3 * n, 1) - ...
                guess(2 * n + 1 : 3 * n - 1, 1)) ...
                * vehicle_param.wheel_base ./ (guess(3 * n + 1 : 4 * n - 1, 1) * dt))));
            
            %% update acceleration, omega
            guess(5 * n + 2 : 6 * n, 1) = ...
                min(vehicle_param.max_acceleration, max(-vehicle_param.max_acceleration, ...
                (guess(3 * n + 2 : 4 * n, 1) - ...
                guess(3 * n + 1 : 4 * n - 1, 1)) / dt));
            
            guess(6 * n + 2 : 7 * n, 1) = ...
                min(vehicle_param.max_acceleration, max(-vehicle_param.max_acceleration, ...
                (guess(4 * n + 2 : 5 * n, 1) - guess(4 * n + 1 : 5 * n - 1, 1)) / dt));
            
            %% update jerk
            guess(7 * n + 2 : 8 * n, 1) = ...
                min(vehicle_param.max_jerk, max(-vehicle_param.max_jerk, ...
                (guess(5 * n + 2 : 6 * n, 1) - guess(5 * n + 1 : 6 * n - 1, 1)) / dt));
            
            %% updata xf, yf, xr, yr
            front_length = 0.75 * vehicle_param.length - vehicle_param.rear_oh;
            rear_length = 0.25 * vehicle_param.length - vehicle_param.rear_oh;
            guess(8 * n + 1 : 9 * n, 1) = ...
                guess(1 : n, 1) + front_length * cos(guess(2 * n + 1 : 3 * n, 1));
            guess(9 * n + 1 : 10 * n, 1) = ...
                guess(n + 1 : 2 * n, 1) + front_length * sin(guess(2 * n + 1 : 3 * n, 1));
            guess(10 * n + 1 : 11 * n, 1) = ...
                guess(1 : n, 1) + rear_length * cos(guess(2 * n + 1 : 3 * n, 1));
            guess(11 * n + 1 : 12 * n, 1) = ...
                guess(n + 1 : 2 * n, 1) + rear_length * sin(guess(2 * n + 1 : 3 * n, 1));
        end
        
        function trajectory = DataTransform(traj)
            dp_config = GetDPPlannerConfig();
            n = dp_config.nfe;
            dt = dp_config.tf / (n - 1);
            trajectory.t = dt * [0 : 1 : n - 1];
            trajectory.x = traj(1 : n);
            trajectory.y = traj(n + 1 : 2 * n);
            trajectory.theta = traj(2 * n + 1 : 3 * n);
            trajectory.v = traj(3 * n + 1 : 4 * n);
            trajectory.delta = traj(4 * n + 1 : 5 * n);
            trajectory.a = traj(5 * n + 1 : 6 * n);
            trajectory.omega = traj(6 * n + 1 : 7 * n);
            trajectory.jerk = traj(7 * n + 1 : 8 * n);
        end
    end
end


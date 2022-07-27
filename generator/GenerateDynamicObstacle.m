classdef GenerateDynamicObstacle < handle
    
    properties
        lane_
        obstacles_ = [];
        time_resolution_ = 0.5;
        survival_time_ = 20.0;
        kEpsilon = 1e-6
    end
    
    methods
        function obj = GenerateDynamicObstacle(lane)
            obj.lane_ = lane;
        end
        
        function obstacles = GetObstacles(obj)
            obstacles = obj.obstacles_;
        end
        
        %% Uniform acceleration model
        function obj = AddAlignedObstacle(obj, type, start_s, start_v, start_a, offset_center)
            switch(type)
                case 'car'
                    width_= 2.2;
                    length_ = 4.5;
                case 'bus'
                    width_ = 2.5;
                    length_ = 12.5;
                otherwise
                    width_ = 2.2;
                    length_ = 4.5;
            end
            start_index = max(1, min(ceil(start_s / obj.lane_.resolution), length(obj.lane_.center_line.s)));
            center_line_pt.x = obj.lane_.center_line.x(start_index);
            center_line_pt.y = obj.lane_.center_line.y(start_index);
            center_line_pt.theta = obj.lane_.center_line.theta(start_index);
            center_pt = obj.PointOffset(center_line_pt, offset_center);
            obstacle = obj.Obstacle(center_pt, length_, width_);
            obstacle.trajectory = obj.GenerateTrajectory(start_index, offset_center, start_v, start_a);
            obj.obstacles_ = [obj.obstacles_, obstacle];
        end

        function obstacle = Obstacle(obj, center_pt, length, width)
            half_length = length / 2.0;
            half_width = width / 2.0;

            pt1.x = center_pt.x + half_length * cos(center_pt.theta) - half_width * sin(center_pt.theta);
            pt1.y = center_pt.y + half_length * sin(center_pt.theta) + half_width * cos(center_pt.theta);
            
            pt2.x = center_pt.x - half_length * cos(center_pt.theta) - half_width * sin(center_pt.theta);
            pt2.y = center_pt.y - half_length * sin(center_pt.theta) + half_width * cos(center_pt.theta);
            
            
            pt3.x = center_pt.x - half_length * cos(center_pt.theta) + half_width * sin(center_pt.theta);
            pt3.y = center_pt.y - half_length * sin(center_pt.theta) - half_width * cos(center_pt.theta);
            
            
            pt4.x = center_pt.x + half_length * cos(center_pt.theta) + half_width * sin(center_pt.theta);
            pt4.y = center_pt.y + half_length * sin(center_pt.theta) - half_width * cos(center_pt.theta);
            
            obstacle.position.x = center_pt.x;
            obstacle.position.y = center_pt.y;
            obstacle.heading = center_pt.theta;
            obstacle.polygon = [pt1, pt2, pt3, pt4];
            obstacle.width = width;
            obstacle.length = length;
        end
        
        function trajectory = GenerateTrajectory(obj, start_index, offset, start_v, start_a)
            trajectory = [];
            center_line_pt.x = obj.lane_.center_line.x(start_index);
            center_line_pt.y = obj.lane_.center_line.y(start_index);
            center_line_pt.theta = obj.lane_.center_line.theta(start_index);
            
            point = obj.PointOffset(center_line_pt, offset);
            trajectory_point.relative_time = 0.0;
            trajectory_point.x = point.x;
            trajectory_point.y = point.y;
            trajectory_point.theta = point.theta;
            trajectory_point.s = obj.lane_.center_line.s(start_index);
            trajectory_point.velocity = start_v;
            trajectory_point.acceleration = start_a;
            trajectory = [trajectory, trajectory_point];
            
            i = 1;
            center_line_length = obj.lane_.center_line.s(end);
            while trajectory_point.s + obj.kEpsilon < center_line_length
                [s, v, a] = obj.VehicleModel(trajectory_point.velocity, trajectory_point.acceleration, obj.time_resolution_);
                trajectory_point.relative_time = i * obj.time_resolution_;
                if trajectory_point.relative_time > obj.survival_time_
                    break;
                end
                trajectory_point.s = trajectory_point.s + s;
                center_point = obj.CalculateCenterLinePointByS(start_index, trajectory_point.s);
                point = obj.PointOffset(center_point, offset);
                trajectory_point.x = point.x;
                trajectory_point.y = point.y;
                trajectory_point.theta = point.theta;
                trajectory_point.velocity = v;
                trajectory_point.acceleration = a;
                trajectory = [trajectory, trajectory_point];
                i = i + 1;
            end
            
        end
        
        function point = PointOffset(obj, center_point, offset)
            point = center_point;
            point.x = center_point.x - offset * sin(center_point.theta);
            point.y = center_point.y + offset * cos(center_point.theta);
        end
        
        function [s, v, a] = VehicleModel(obj, v0, a0, t)
            if t < obj.kEpsilon
                s = 0.0;
                v = v0;
                a = a0;
                return;
            end
            
            dec = min(obj.kEpsilon, a0 + obj.kEpsilon);
            stop_t = v0 / abs(dec);
            if t < stop_t 
                s = v0 * t + 0.5 * a0 * t^2;
                v = v0 + a0 * t;
                a = a0;
            else
                s = 0.5 * v0 ^ 2 / abs(a0 + obj.kEpsilon);
                v = 0.0;
                a = 0.0;
            end
        end
        
        function point = CalculateCenterLinePointByS(obj, start_index, s)
            index = start_index;
            for i = start_index : 1 : length(obj.lane_.center_line.s)
                if obj.lane_.center_line.s(i) >= s
                    index = i;
                    break;
                end
            end
            
            next_index = index;
            if index < length(obj.lane_.center_line.s)
                next_index = index + 1;
            end
            pt_index.x = obj.lane_.center_line.x(index);
            pt_index.y = obj.lane_.center_line.y(index);
            pt_index.theta = obj.lane_.center_line.theta(index);
            pt_index.s = obj.lane_.center_line.s(index);
            
            pt_next_index.x = obj.lane_.center_line.x(next_index);
            pt_next_index.y = obj.lane_.center_line.y(next_index);
            pt_next_index.theta = obj.lane_.center_line.theta(next_index);
            pt_next_index.s = obj.lane_.center_line.s(next_index);
            
            ratio = (s - pt_index.s) / (pt_next_index.s - pt_index.s);
            
            point.x = pt_index.x + ratio * (pt_next_index.x - pt_index.x);
            point.y = pt_index.y + ratio * (pt_next_index.y - pt_index.y);
            point.theta = pt_index.theta + ratio * (pt_next_index.theta - pt_index.theta);       
        end
            
    end
end


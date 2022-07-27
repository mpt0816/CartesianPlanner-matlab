classdef Environment < handle
    
    properties
        reference_line_
        road_width_
        road_length_
        road_barrier_
        static_obstacles_
        dynamic_obstacles_
        vehicle_config_
        road_left_boundary_;
        road_right_boundary_;
        kEpsilon = 1e-6
    end
    
    methods
        function obj = Environment(scenario)
            obj.reference_line_ = scenario.lane.center_line;
            obj.road_width_ = scenario.lane.lane_width;
            obj.road_length_ = obj.reference_line_.s(end);
            obj.vehicle_config_ = GetVehicleParams();
            obj.road_left_boundary_ = scenario.lane.left_boundary;
            obj.road_right_boundary_ = scenario.lane.right_boundary;
            obj.HandleRoadBarry(scenario.lane.left_boundary, scenario.lane.right_boundary);
            obj.HandleStaticObstacles(scenario.static_obstacles);
            obj.HandleDynamicObstacles(scenario.dynamic_obstacles);
        end
        
        function ref_line = GetReferenceLine(obj)
            ref_line = obj.reference_line_;
        end
        
        function road_width = GetRoadWidth(obj)
            road_width = obj.road_width_;
        end
        
        function road_length = GetRoadLength(obj)
            road_length = obj.road_length_;
        end
        
        function road_left_boundary = GetRoadLeftBoundary(obj)
            road_left_boundary = obj.road_left_boundary_;
        end
        
        function road_right_boundary = GetRoadRightBoundary(obj)
            road_right_boundary = obj.road_right_boundary_;
        end
        
        function obj = HandleRoadBarry(obj, left_boundary, right_boundary)
            num = length(left_boundary.x);
            temp_barrier.x = [left_boundary.x; right_boundary.x];
            temp_barrier.y = [left_boundary.y; right_boundary.y];
            
            [~, index] = sort(temp_barrier.x, 'ascend');
            obj.road_barrier_.x = temp_barrier.x(index);
            obj.road_barrier_.y = temp_barrier.y(index);
        end
        
        function obj = HandleStaticObstacles(obj, obstacles)
            obj.static_obstacles_ = [];
            for obstacle = obstacles
                x = obstacle.position.x;
                y = obstacle.position.y;
                box = Box2d(x, y, obstacle.heading, obstacle.length, obstacle.width);
                obj.static_obstacles_ = [obj.static_obstacles_, box];
            end
        end
        
        function obj = HandleDynamicObstacles(obj, obstacles)
            obj.dynamic_obstacles_ = [];
            id = 1;
            for obstacle = obstacles
                info = [];
                for pt = obstacle.trajectory
                    pair.relative_time = pt.relative_time;
                    pair.box = Box2d(pt.x, pt.y, pt.theta, obstacle.length, obstacle.width);
                    info = [info, pair];
                end
                dynamic_obstacle.id = id;
                dynamic_obstacle.info = info;
                obj.dynamic_obstacles_ = [obj.dynamic_obstacles_, dynamic_obstacle];
                id = id + 1;
            end
        end
        
        function obstacles = QueryDynamicObstacles(obj, time)
            obstacles = [];
            for dynamic_obstacle = obj.dynamic_obstacles_
                info = dynamic_obstacle.info;
                for pair = info
                    if pair.relative_time > time + 0.5 - obj.kEpsilon
                        break;
                    end
                    if pair.relative_time > time - obj.kEpsilon
                        obstacles = [obstacles, pair.box];
                    end
                end
            end
        end
        
        function out = CheckStaticCollision(obj, veh_box)
            for obs_box = obj.static_obstacles_
                if veh_box.HasOverlap(obs_box)
                    out = true;
                    return;
                end
            end
            
            if isempty(obj.road_barrier_)
                out = false;
                return;
            end
            
            num = length(obj.road_barrier_.x);
            for i = 1 : 1 : num
                if obj.road_barrier_.x(i) > veh_box.max_x()
                    break;
                end
                
                if obj.road_barrier_.x(i) < veh_box.min_x()
                    continue;
                end
                
                if veh_box.IsPointIn(obj.road_barrier_.x(i), obj.road_barrier_.y(i))
                    out = true;
                    return;
                end
            end
            out = false;
        end
        
        function out = CheckDynamicCollision(obj, time, veh_box)
            obstacles = obj.QueryDynamicObstacles(time);
            for box = obstacles
                if veh_box.HasOverlap(box)
                    out = true;
                    return;
                end
            end
            out = false;
        end
        
        function out = CheckCollision(obj, time, veh_box)
            if obj.CheckDynamicCollision(time, veh_box)
                out = true;
                return;
            end
            
            out = obj.CheckStaticCollision(veh_box);
        end
        
        function out = CheckVehicleCollision(obj, time, x, y, heading)
            [center_x, center_y] = Utils.GetVehicleCenterPosition(x, y, heading, obj.vehicle_config_);
            veh_box = Box2d(center_x, center_y, heading, obj.vehicle_config_.length, obj.vehicle_config_.width);
            out = obj.CheckCollision(time, veh_box);
        end
        
        function out = CheckVehicleDiscCollision(obj, time, x, y, heading)
            [front_x, front_y, rear_x, rear_y] = Utils.GetVehicleDiscPositions(x, y, heading, obj.vehicle_config_);
            front_box = Box2d(front_x, front_y, 0.0, 2 * obj.vehicle_config_.radius, 2 * obj.vehicle_config_.radius);
            if obj.CheckCollision(time, front_box)
                out = true;
                return;
            end
            
            rear_box = Box2d(rear_x, rear_y, 0.0, 2 * obj.vehicle_config_.radius, 2 * obj.vehicle_config_.radius);
            out = obj.CheckCollision(time, rear_box);
        end
        
        function point = EvaluatePointByS(obj, s)
            if s < obj.reference_line_.s(1) + obj.kEpsilon
                point.x = obj.reference_line_.x(1);
                point.y = obj.reference_line_.y(1);
                point.s = obj.reference_line_.s(1);
                point.theta = obj.reference_line_.theta(1);
                point.kappa = obj.reference_line_.kappa(1);
                point.dkappa = obj.reference_line_.dkappa(1);
                return;
            end
            
            if s > obj.reference_line_.s(end) - obj.kEpsilon
                point.x = obj.reference_line_.x(end);
                point.y = obj.reference_line_.y(end);
                point.s = obj.reference_line_.s(end);
                point.theta = obj.reference_line_.theta(end);
                point.kappa = obj.reference_line_.kappa(end);
                point.dkappa = obj.reference_line_.dkappa(end);
                return;
            end
            num = length(obj.reference_line_.x);
            index = 1;
            for i = 1 : 1 : num
                if obj.reference_line_.s(i) >= s
                    index = i;
                    break;
                end
            end
            front_index = max(1, index - 1);
            front_point.x = obj.reference_line_.x(front_index);
            front_point.y = obj.reference_line_.y(front_index);
            front_point.s = obj.reference_line_.s(front_index);
            front_point.theta = obj.reference_line_.theta(front_index);
            front_point.kappa = obj.reference_line_.kappa(front_index);
            front_point.dkappa = obj.reference_line_.dkappa(front_index);
            
            back_point.x = obj.reference_line_.x(index);
            back_point.y = obj.reference_line_.y(index);
            back_point.s = obj.reference_line_.s(index);
            back_point.theta = obj.reference_line_.theta(index);
            back_point.kappa = obj.reference_line_.kappa(index);
            back_point.dkappa = obj.reference_line_.dkappa(index);
            point = obj.LinearInterpolateByS(front_point, back_point, s);
        end
        
        function point = LinearInterpolateByS(obj, front_point, back_point, s)
            if abs(front_point.s - back_point.s) < obj.kEpsilon
                point = front_point;
                return;
            end
            
            ratio = (s - front_point.s) / (back_point.s - front_point.s);
            point.s = s;
            point.x = (1 - ratio) * front_point.x + ratio * back_point.x;
            point.y = (1 - ratio) * front_point.y + ratio * back_point.y;
            point.theta = (1 - ratio) * front_point.theta + ratio * back_point.theta;
            point.theta = Math.NormalizeAngle(point.theta);
            point.kappa = (1 - ratio) * front_point.kappa + ratio * back_point.kappa;
            point.dkappa = (1 - ratio) * front_point.dkappa + ratio * back_point.dkappa;
        end
        
        function [s, l] = GetSLProjection(obj, x, y)
            num = length(obj.reference_line_.x);
            min_dis = inf;
            index = 1;
            for i = 1 : 1 : num
                dx = obj.reference_line_.x(i) - x;
                dy = obj.reference_line_.y(i) - y;
                dis = dx^2 + dy^2;
                if dis < min_dis
                    min_dis = dis;
                    index = i;
                end
            end
            front_index = max(1, index - 1);
            back_index = min(num, index + 1);
            
            v0x = x - obj.reference_line_.x(front_index);
            v0y = y - obj.reference_line_.y(front_index);
            v1x = obj.reference_line_.x(back_index) - obj.reference_line_.x(front_index);
            v1y = obj.reference_line_.y(back_index) - obj.reference_line_.y(front_index);
            
            proj = (v0x * v1x + v0y * v1y) / hypot(v1x, v1y);
            norm = (v1x * v0y - v0x * v1y) / hypot(v1x, v1y);
            
            s = obj.reference_line_.s(front_index) + proj;
            l = norm;
        end
        
        function [x, y] = GetCartesian(obj, s, l)
            ref_pt = obj.EvaluatePointByS(s);
            x = ref_pt.x - l * sin(ref_pt.theta);
            y = ref_pt.y + l * cos(ref_pt.theta);
        end
        
        function static_obstacles = static_obstacles(obj)
            static_obstacles = obj.static_obstacles_;
        end
        
        function dynamic_obstacles = dynamic_obstacles(obj)
            dynamic_obstacles = obj.dynamic_obstacles_;
        end
    end
end


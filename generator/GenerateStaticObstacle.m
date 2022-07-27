classdef GenerateStaticObstacle < handle
    
    properties
        width_ = 2.2
        length_ = 4.5
        type_ = 'car'
        num_ = 1
        offset_center_ = 0.0
        lane_
        obstacles_ = [];
        kEpsilon = 1e-6
    end
    
    methods
        function obj = GenerateStaticObstacle(lane, type, num, offset_center)
            obj.lane_ = lane;
            obj.type_ = type;
            obj.num_ = num;
            obj.offset_center_ = offset_center;
            switch(obj.type_)
                case 'car'
                    obj.width_ = 2.2;
                    obj.length_ = 4.5;
                case 'bus'
                    obj.width_ = 2.5;
                    obj.length_ = 12.5;
                otherwise
                    obj.width_ = 2.2;
                    obj.length_ = 4.5;
            end
            obj.obstacles_ = obj.Obstacles();
        end
        
        function obstacles = Obstacles(obj)
            lane_length = obj.lane_.center_line.s(end);
            segment_s = lane_length / (obj.num_ + 1);
            
            obstacles = [];
            if obj.num_ < obj.kEpsilon
                return
            end
            
            sign_coef = 1;
            for i = 1 : 1 : obj.num_
                s = segment_s * i;
                for j = 1 : 1 : length(obj.lane_.center_line.s)
                    if obj.lane_.center_line.s(j) >= s
                        center_line_pt.x = obj.lane_.center_line.x(j);
                        center_line_pt.y = obj.lane_.center_line.y(j);
                        center_line_pt.theta = obj.lane_.center_line.theta(j);
                        break;
                    end
                end
                offset = obj.lane_.lane_width / 2.0 - obj.offset_center_  + obj.width_ / 2.0;
                offset = sign_coef * offset;
                obs_center_pt = obj.PointOffset(center_line_pt, offset);
                obstacle = obj.Obstacle(obs_center_pt);
                obstacles = [obstacles, obstacle];
                sign_coef = -1 * sign_coef;
            end  
        end
        
        function point = PointOffset(obj, center_point, offset)
            point = center_point;
            point.x = center_point.x - offset * sin(center_point.theta);
            point.y = center_point.y + offset * cos(center_point.theta);
        end
        
        function obstacle = Obstacle(obj, center_pt)
            half_length = obj.length_ / 2.0;
            half_width = obj.width_ / 2.0;

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
            obstacle.type = obj.type_;
            obstacle.width = obj.width_;
            obstacle.length = obj.length_;
        end
        
        function obstacles = GetObstacles(obj)
            obstacles = obj.obstacles_;
        end

        
    end
end


classdef Math < handle
    
    methods
        function obj = Math()
        end
    end
    
    methods(Static)
        function norm = NormalizeAngle(angle)
            norm = angle;
            %             norm = mod(angle + pi, 2.0 * pi);
            %             if norm < 0.0
            %                 norm = norm + pi;
            %             end
            %             norm = norm - pi;
        end
        
        function angle = AngleDiff(from,to)
            angle = Math.NormalizeAngle(to - from);
        end
        
        function outputArg = DistanceSquare(pt1, pt2)
            outputArg = (pt1.x - pt2.x)^2 + (pt1.y - pt2.y)^2;
        end
        
        function outputArg = Distance(pt1, pt2)
            outputArg = sqrt((pt1.x - pt2.x)^2 + (pt1.y - pt2.y)^2);
        end
    end
end


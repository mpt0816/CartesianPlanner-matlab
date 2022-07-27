classdef GenerateSpiralPath < handle
    properties
        resolution_ = 0.5
    end
    
    methods
        function obj = GenerateSpiralPath()
        end
        
        function obj = SetResolution(obj, resolution)
            obj.resolution_ = resolution;
        end
        
        function path = GeneratePathWithTwoPoint(obj, start_pt, end_pt)
            
            delta_s = end_pt.s - start_pt.s;
            angle_diff = Math.AngleDiff(start_pt.theta, end_pt.theta);
            spiral_curve = QuinticSpiralCurve1d(start_pt.theta, start_pt.kappa, start_pt.dkappa, ...
                start_pt.theta + angle_diff, end_pt.kappa, end_pt.dkappa, delta_s);
            num_of_points = ceil(delta_s / obj.resolution_) + 1;
            
            path.x = zeros(num_of_points + 1, 1);
            path.y = zeros(num_of_points + 1, 1);
            path.s = zeros(num_of_points + 1, 1);
            path.theta = zeros(num_of_points + 1, 1);
            path.kappa = zeros(num_of_points + 1, 1);
            path.dkappa = zeros(num_of_points + 1, 1);
            
            path.x(1, 1) = start_pt.x;
            path.y(1, 1) = start_pt.y;
            path.s(1, 1) = start_pt.s;
            path.theta(1, 1) = start_pt.theta;
            path.kappa(1, 1) = start_pt.kappa;
            path.dkappa(1, 1) = start_pt.dkappa;
            
            for i = 1 : 1 : num_of_points
                inter_s = (delta_s / num_of_points) * i;
                path.x(i + 1, 1) = start_pt.x + spiral_curve.ComputeCartesianDeviationX(inter_s);
                path.y(i + 1, 1) = start_pt.y + spiral_curve.ComputeCartesianDeviationY(inter_s);
                path.s(i + 1, 1) = start_pt.s + inter_s;
                path.theta(i + 1, 1) = Math.NormalizeAngle(spiral_curve.Evaluate(0, inter_s));
                path.kappa(i + 1, 1) = spiral_curve.Evaluate(1, inter_s);
                path.dkappa(i + 1, 1) = spiral_curve.Evaluate(2, inter_s);
            end
        end
        
        function path = GeneratePathWithPointList(obj, pts)
            path.x = [];
            path.y = [];
            path.s = [];
            path.theta = [];
            path.kappa = [];
            path.dkappa = [];
            num_of_pts = length(pts);
            for i = 1 : 1 : num_of_pts - 1
                start = pts(i);
                if ~isempty(path.x)
                    start.x = path.x(end);
                    start.y = path.y(end);
                    start.theta = path.theta(end);
                    start.s = path.s(end);
                    start.kappa = path.kappa(end);
                    start.dkappa = path.dkappa(end);
                end
                segment = obj.GeneratePathWithTwoPoint(start, pts(i + 1));
                path.x = [path.x; segment.x];
                path.y = [path.y; segment.y];
                path.s = [path.s; segment.s];
                path.theta = [path.theta; segment.theta];
                path.kappa = [path.kappa; segment.kappa];
                path.dkappa = [path.dkappa; segment.dkappa];
            end
            path = obj.RemoveDuplicates(path);
        end
        
        function new_path = RemoveDuplicates(obj, path)
            kDuplicatedPointsEpsilon = 1e-6;
            limit = kDuplicatedPointsEpsilon * kDuplicatedPointsEpsilon;
            new_path.x = [];
            new_path.y = [];
            new_path.s = [];
            new_path.theta = [];
            new_path.kappa = [];
            new_path.dkappa = [];
            
            for i = 1 : 1 : length(path.x)
                if isempty(new_path.x)
                    new_path.x = [new_path.x; path.x(i)];
                    new_path.y = [new_path.y; path.y(i)];
                    new_path.s = [new_path.s; path.s(i)];
                    new_path.theta = [new_path.theta; path.theta(i)];
                    new_path.kappa = [new_path.kappa; path.kappa(i)];
                    new_path.dkappa = [new_path.dkappa; path.dkappa(i)];
                    continue;
                end
                s2 = (path.s(i) - new_path.s(end, 1)) ^ 2;
                dis2 = (path.x(i) - new_path.x(end, 1)) ^ 2 + (path.y(i) - new_path.y(end, 1)) ^ 2;
                if dis2 > limit || s2 > limit
                    new_path.x = [new_path.x; path.x(i)];
                    new_path.y = [new_path.y; path.y(i)];
                    new_path.s = [new_path.s; path.s(i)];
                    new_path.theta = [new_path.theta; path.theta(i)];
                    new_path.kappa = [new_path.kappa; path.kappa(i)];
                    new_path.dkappa = [new_path.dkappa; path.dkappa(i)];
                end
            end
        end
        
    end
end


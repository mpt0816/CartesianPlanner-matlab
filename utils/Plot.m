classdef Plot < handle
    
    methods
        function obj = Plot()
        end
    end
    
    methods(Static)
        
        function MakeGif(filename, index)
            f = getframe(gcf);
            imind = frame2im(f);
            [imind,cm] = rgb2ind(imind, 256);
            if index==1
                imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',0.001);
            else
                imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',0.001);
            end
        end
        
        function region = PlotEnvironment(scenario)
            region = Plot.PlotLane(scenario.lane);
            hold on;
            Plot.PlotStaticObstacles(scenario.static_obstacles);
        end
        
        function region = PlotLane(lane)
            if isempty(lane.center_line)
                return;
            end
            road_color = [190 190 190] / 255; %% grey
            fill_x = [lane.left_boundary.x; flip(lane.right_boundary.x)];
            fill_y = [lane.left_boundary.y; flip(lane.right_boundary.y)];
            p1 = fill(fill_x, fill_y, road_color);
            set(p1,'edgealpha',0);
            set(p1, 'handlevisibility', 'off');
            hold on;
            
            plot(lane.center_line.x, lane.center_line.y, 'k--', 'LineWidth', 0.5);
            hold on;
            
            boundary = plot(lane.left_boundary.x, lane.left_boundary.y, 'k-', 'LineWidth', 1.0);
            set(boundary, 'handlevisibility', 'off');
            hold on;
            
            boundary = plot(lane.right_boundary.x, lane.right_boundary.y, 'k-', 'LineWidth', 1.0);
            set(boundary, 'handlevisibility', 'off');
            
            left_shoulder_region = Plot.PlotShoulder(lane.left_shoulder);
            right_shoulder_region = Plot.PlotShoulder(lane.right_shoulder);
            
            region.min_x = min([min(lane.center_line.x), min(lane.left_boundary.x), min(lane.right_boundary.x), left_shoulder_region.min_x, right_shoulder_region.min_x]);
            region.max_x = max([max(lane.center_line.x), max(lane.left_boundary.x), max(lane.right_boundary.x), left_shoulder_region.max_x, right_shoulder_region.max_x]);
            region.min_y = min([min(lane.center_line.y), min(lane.left_boundary.y), min(lane.right_boundary.y), left_shoulder_region.min_y, right_shoulder_region.min_y]);
            region.max_y = max([max(lane.center_line.y), max(lane.left_boundary.y), max(lane.right_boundary.y), left_shoulder_region.max_y, right_shoulder_region.max_y]);
            
            axis equal;
            xlabel('x(m)');
            ylabel('y(m)');
        end
        
        function region = PlotShoulder(shoulder)
            left_boundary = shoulder.left_boundary;
            right_boundary = shoulder.right_boundary;
            
            if isempty(left_boundary) || isempty(right_boundary)
                return;
            end
            
            region.min_x = min([min(left_boundary.x), min(right_boundary.x)]);
            region.max_x = max([max(left_boundary.x), max(right_boundary.x)]);
            region.min_y = min([min(left_boundary.y), min(right_boundary.y)]);
            region.max_y = max([max(left_boundary.y), max(right_boundary.y)]);
            
            shoulder_color = [105 105 105] / 255; %% grey
            fill_x = [left_boundary.x; flip(right_boundary.x)];
            fill_y = [left_boundary.y; flip(right_boundary.y)];
            h = fill(fill_x, fill_y, shoulder_color);
            set(h,'edgealpha',0);
            set(h, 'handlevisibility', 'off');
            
        end
        
        function PlotStaticObstacles(obstacles)
            
            if isempty(obstacles)
                return;
            end
            
            for obstacle = obstacles
                Plot.PlotStaticObstacle(obstacle);
                hold on;
            end
            
        end
        
        function PlotDynamicObstacles(obstacles)
            
            if isempty(obstacles)
                return;
            end
            
            for obstacle = obstacles
                Plot.PlotDynamicObstacle(obstacle);
            end
            
        end
        
        function PlotStaticObstacle(obstacle)
            corners = obstacle.polygon;
            x_list = [];
            y_list = [];
            for corner = corners
                x_list = [x_list, corner.x];
                y_list = [y_list, corner.y];
            end
            obstacle_color = [205 51 51] / 255;  %% Brown;
            p_obstalce = fill(x_list, y_list, obstacle_color);
            set(p_obstalce, 'handlevisibility', 'off');
        end
        
        function PlotDynamicObstacle(obstacle)
            trajectory = obstacle.trajectory;
            num = length(trajectory);
            x = zeros(1, num);
            y = zeros(1, num);
            heading = zeros(1, num);
            t = zeros(1, num);
            s = zeros(1, num);
            v = zeros(1, num);
            a = zeros(1, num);
            
            for i = 1 : 1 : num
                point = trajectory(i);
                x(1, i) = point.x;
                y(1, i) = point.y;
                heading(1, i) = point.theta;
                t(1, i) = point.relative_time;
                s(1, i) = point.s;
                v(1, i) = point.velocity;
                a(1, i) = point.acceleration;
            end
            figure;
            plot(x, y, 'k-', 'LineWidth', 1.0);
            xlabel('x(m)');
            ylabel('y(m)');
            legend('Trajectory');
            
            figure;
            subplot(4, 1, 1);
            plot(s, heading, 'k-', 'LineWidth', 1.0);
            xlabel('s(m)');
            ylabel('theta(rad)');
            legend('Heading');
            
            subplot(4, 1, 2);
            plot(t, s, 'k-', 'LineWidth', 1.0);
            xlabel('t(s)');
            ylabel('s(m)');
            legend('Displacement');
            
            subplot(4, 1, 3);
            plot(t, v, 'k-', 'LineWidth', 1.0);
            xlabel('t(s)');
            ylabel('v(m/s)');
            legend('Velocity');
            
            subplot(4, 1, 4);
            plot(t, a, 'k-', 'LineWidth', 1.0);
            xlabel('t(s)');
            ylabel('a(m/s2)');
            legend('Acceleration');
        end
        
        function PlotSafeCorridor(env, traj, corridor)
            
            dp_config = GetDPPlannerConfig();
            dt = dp_config.tf / (dp_config.nfe - 1);
            t_total = dp_config.tf;
            
            %% plot Road
            road_left_boundary = env.GetRoadLeftBoundary();
            road_right_boundary = env.GetRoadRightBoundary();
            road_color = [190 190 190] / 255; %% grey
            road_boundary_x = [road_left_boundary.x; flip(road_right_boundary.x)];
            road_boundary_y = [road_left_boundary.y; flip(road_right_boundary.y)];
            fill3(road_boundary_x, road_boundary_y, zeros(length(road_boundary_x), 1), road_color, 'edgealpha', 0);
            hold on;
            
            num_boundary = length(road_left_boundary.x);
            plot3(road_left_boundary.x, road_left_boundary.y, zeros(1, num_boundary), '-', 'Color', 'k', 'LineWidth', 1.0);
            hold on;
            plot3(road_right_boundary.x, road_right_boundary.y, zeros(1, num_boundary), '-', 'Color', 'k', 'LineWidth', 1.0);
            hold on;
            plot3(env.GetReferenceLine.x, env.GetReferenceLine.y, zeros(1, num_boundary), '--', 'Color', 'w', 'LineWidth', 1.0);
            hold on;
            
            
            %% traj
            traj_t = dt * [0 : 1 : dp_config.nfe - 1];
            plot3(traj.x, traj.y, traj_t, 'r', 'LineWidth', 1.5);
            hold on;
            
            %% static obstacles
            for i = 1 : 1 : dp_config.nfe
                obstacle_t = (i - 1) * dt * ones(1, 5);
                for static_obstacle = env.static_obstacles()
                    
                    obstacle_x = zeros(1, 5);
                    obstacle_y = zeros(1, 5);
                    corners = static_obstacle.corners();
                    for j = 1 : 1 : 4
                        pt = corners(j);
                        obstacle_x(1, j) = pt.x;
                        obstacle_y(1, j) = pt.y;
                    end
                    pt = corners(1);
                    obstacle_x(1, 5) = pt.x;
                    obstacle_y(1, 5) = pt.y;
                    plot3(obstacle_x, obstacle_y, obstacle_t, 'Color', Plot.GetColor(obstacle_t(1), t_total));
                    hold on;
                end
            end
            
            %% dynamic obstacles
            for dynamic_obstacle = env.dynamic_obstacles()
                obstacle_t = zeros(1, 5);
                obstacle_x = zeros(1, 5);
                obstacle_y = zeros(1, 5);
                info = dynamic_obstacle.info;
                for pair = info
                    if pair.relative_time > t_total
                        break;
                    end
                    corners = pair.box.corners();
                    for j = 1 : 1 : 4
                        pt = corners(j);
                        obstacle_t(1, j) = pair.relative_time;
                        obstacle_x(1, j) = pt.x;
                        obstacle_y(1, j) = pt.y;
                    end
                    pt = corners(1);
                    obstacle_t(1, 5) = pair.relative_time;
                    obstacle_x(1, 5) = pt.x;
                    obstacle_y(1, 5) = pt.y;
                    plot3(obstacle_x, obstacle_y, obstacle_t, 'Color', Plot.GetColor(obstacle_t(1), t_total));
                    hold on;
                end
            end
            
            %% safe corridor
            for i = 1 : 1 : dp_config.nfe
                relative_time = (i - 1) * dt;
                
                front_corridor_t = relative_time * ones(1, 5);
                front_corridor_x = [corridor.f_max_x(i), corridor.f_min_x(i), corridor.f_min_x(i), corridor.f_max_x(i), corridor.f_max_x(i)];
                front_corridor_y = [corridor.f_max_y(i), corridor.f_max_y(i), corridor.f_min_y(i), corridor.f_min_y(i), corridor.f_max_y(i)];
                
                rear_corridor_t = relative_time * ones(1, 5);
                rear_corridor_x = [corridor.r_max_x(i), corridor.r_min_x(i), corridor.r_min_x(i), corridor.r_max_x(i), corridor.r_max_x(i)];
                rear_corridor_y = [corridor.r_max_y(i), corridor.r_max_y(i), corridor.r_min_y(i), corridor.r_min_y(i), corridor.r_max_y(i)];
                
                plot3(front_corridor_x, front_corridor_y, front_corridor_t, 'Color', [Plot.GetColor(front_corridor_t(1), t_total), 0.4], 'LineWidth', 0.5);
                hold on;
                plot3(rear_corridor_x, rear_corridor_y, rear_corridor_t, 'Color',[Plot.GetColor(rear_corridor_t(1), t_total), 0.4], 'LineWidth', 0.5);
                hold on;
            end
            
            xlabel('x(m)');
            ylabel('y(m)');
            zlabel('time(s)');
            axis equal;
        end
        
        function PlotResult(env, traj)
            
            dp_config = GetDPPlannerConfig();
            dt = dp_config.tf / (dp_config.nfe - 1);
            t_total = dp_config.tf;
            
            figure;
            %% Plot Road
            road_left_boundary = env.GetRoadLeftBoundary();
            road_right_boundary = env.GetRoadRightBoundary();
            road_color = [190 190 190] / 255; %% grey
            road_boundary_x = [road_left_boundary.x; flip(road_right_boundary.x)];
            road_boundary_y = [road_left_boundary.y; flip(road_right_boundary.y)];
            fill3(road_boundary_x, road_boundary_y, zeros(length(road_boundary_x), 1), road_color, 'edgealpha', 0);
            hold on;
  
            num_boundary = length(road_left_boundary.x);
            plot3(road_left_boundary.x, road_left_boundary.y, zeros(1, num_boundary), '-', 'Color', 'k', 'LineWidth', 1.0);
            hold on;
            plot3(road_right_boundary.x, road_right_boundary.y, zeros(1, num_boundary), '-', 'Color', 'k', 'LineWidth', 1.0);
            hold on;
            plot3(env.GetReferenceLine.x, env.GetReferenceLine.y, zeros(1, num_boundary), '--', 'Color', 'w', 'LineWidth', 1.0);
            hold on;
            
            %% static obstacles
            for i = 1 : 1 : dp_config.nfe
                obstacle_t = (i - 1) * dt * ones(1, 5);
                for static_obstacle = env.static_obstacles()
                    
                    obstacle_x = zeros(1, 5);
                    obstacle_y = zeros(1, 5);
                    corners = static_obstacle.corners();
                    for j = 1 : 1 : 4
                        pt = corners(j);
                        obstacle_x(1, j) = pt.x;
                        obstacle_y(1, j) = pt.y;
                    end
                    pt = corners(1);
                    obstacle_x(1, 5) = pt.x;
                    obstacle_y(1, 5) = pt.y;
                    plot3(obstacle_x, obstacle_y, obstacle_t, 'Color', Plot.GetColor(obstacle_t(1), t_total));
                    hold on;
                end
            end
            
            %% dynamic obstacles
            for dynamic_obstacle = env.dynamic_obstacles()
                obstacle_t = zeros(1, 5);
                obstacle_x = zeros(1, 5);
                obstacle_y = zeros(1, 5);
                info = dynamic_obstacle.info;
                for pair = info
                    if pair.relative_time > t_total
                        break;
                    end
                    corners = pair.box.corners();
                    for j = 1 : 1 : 4
                        pt = corners(j);
                        obstacle_t(1, j) = pair.relative_time;
                        obstacle_x(1, j) = pt.x;
                        obstacle_y(1, j) = pt.y;
                    end
                    pt = corners(1);
                    obstacle_t(1, 5) = pair.relative_time;
                    obstacle_x(1, 5) = pt.x;
                    obstacle_y(1, 5) = pt.y;
                    plot3(obstacle_x, obstacle_y, obstacle_t, 'Color', Plot.GetColor(obstacle_t(1), t_total));
                    hold on;
                end
            end
            
             %% traj
            traj_t = dt * [0 : 1 : dp_config.nfe - 1];
            plot3(traj.x, traj.y, traj_t, 'r', 'LineWidth', 1.5);
            hold on;
            
            %% vehicle project
            for i = 1 : 1 : dp_config.nfe
                pt.x = traj.x(i);
                pt.y = traj.y(i);
                pt.theta = traj.theta(i);
                corners = Utils.CalculateConersFromRearPoint(pt);
                
                plot3([corners(1).x, corners(2).x, corners(3).x, corners(4).x, corners(1).x], ...
                    [corners(1).y, corners(2).y, corners(3).y, corners(4).y, corners(1).y], ...
                    traj_t(i) * ones(1, 5), 'Color', [Plot.GetColor(traj_t(i), t_total), 0.5], 'LineWidth', 0.5);
            end
            
            xlabel('x(m)');
            ylabel('y(m)');
            zlabel('time(s)');
            axis equal;
%             axis off;
            
            figure;
            subplot(4,2,1);
            plot(traj.x, traj.y, 'k-', 'LineWidth', 1.0);
            xlabel('x(m)'); ylabel('y(m)');
            subplot(4,2,2);
            plot(traj.t, traj.theta, 'k-', 'LineWidth', 1.0);
            xlabel('time(s)'); ylabel('heading(rad)');
            subplot(4,2,3);
            plot(traj.t, traj.v, 'k-', 'LineWidth', 1.0);
            xlabel('time(s)'); ylabel('velocity(m/s)');
            subplot(4,2,4);
            plot(traj.t, traj.delta, 'k-', 'LineWidth', 1.0);
            xlabel('time(s)'); ylabel('delta(rad)');
            subplot(4,2,5);
            plot(traj.t, traj.a, 'k-', 'LineWidth', 1.0);
            xlabel('time(s)'); ylabel('acceleration(m/s2)');
            subplot(4,2,6);
            plot(traj.t, traj.omega, 'k-', 'LineWidth', 1.0);
            xlabel('time(s)'); ylabel('omega(rad/s)');
            subplot(4,2,7);
            plot(traj.t, traj.jerk, 'k-', 'LineWidth', 1.0);
            xlabel('time(s)'); ylabel('jerk(m/s3)');
        end
        
        function color = GetColor(dt, total)
            map = turbo;
            index = floor(255 * dt / total);
            index = min(255, max(1, index));
            color = map(index, :);
            
        end
    end
end


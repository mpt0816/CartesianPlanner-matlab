classdef DpPlanner < handle
    
    properties
        kInf = inf;
        kNInf = -inf;
        
        NT = 5;
        NS = 7;
        NL = 9;
    end
    
    properties
        env_;
        config_;
        vehicle_params_;
        nseg_;  %% the number of point in every segment, int
        unit_time_;
        station_max_;
        
        time_array_;
        station_array_;
        lateral_array_;
        
        start_state_;
        state_space_;
        
        safe_margin_;
        kEpsilon = 1e-6;
    end
    
    methods
        function obj = DpPlanner(env)
            obj.env_ = env;
            obj.config_ = GetDPPlannerConfig();
            obj.vehicle_params_ = GetVehicleParams();
            
            obj.nseg_ = floor(obj.config_.nfe / obj.NT);
            obj.unit_time_ = obj.config_.tf / obj.NT;
            obj.station_max_ = obj.env_.GetRoadLength();
            
            obj.time_array_ = linspace(obj.unit_time_, obj.config_.tf, obj.NT);
            obj.station_array_ = linspace(0.0, obj.unit_time_ * obj.vehicle_params_.max_velocity, obj.NS);
            obj.lateral_array_ = linspace(0, 1, obj.NL);
            obj.safe_margin_ = obj.vehicle_params_.width / 2;
            obj.state_space_ = repmat(struct('cost', obj.kInf, 'current_s', obj.kNInf, 'parent_s_ind', -1, 'parent_l_ind', -1), obj.NT, obj.NS, obj.NL);
        end
        
        function path = Plan(obj, start_state)
            path = [];
            [obj.start_state_.s, obj.start_state_.l] = obj.env_.GetSLProjection(start_state.x, start_state.y);
            
            %% reset state space
            obj.state_space_ = repmat(struct('cost', obj.kInf, 'current_s', obj.kNInf, 'parent_s_ind', 0, 'parent_l_ind', 0), obj.NT, obj.NS, obj.NL);
            
            %% evaluate first layer
            for i = 1 : 1 : obj.NS
                for j = 1 : 1 : obj.NL
                    [s, cost] = obj.GetCost(struct('t', 0, 's', 0, 'l', 0), struct('t', 1, 's', i, 'l', j));
                    obj.state_space_(1, i, j).current_s = s;
                    obj.state_space_(1, i, j).cost = cost;
                end
            end
            
            %% dynamic programming
            for i = 1 : 1 : obj.NT - 1
                for j = 1 : 1 : obj.NS
                    for k = 1 : 1 : obj.NL
                        parent_ind = struct('t', i, 's', j, 'l', k);
                        parent_cost = obj.state_space_(i, j, k).cost;
                        if parent_cost >= obj.config_.weight_obstacle - obj.kEpsilon
                            continue;
                        end
                        
                        for m = 1 : 1 : obj.NS
                            for n = 1 : 1 : obj.NL
                                current_ind = struct('t', i + 1, 's', m, 'l', n);
                                [s, delta_cost] = obj.GetCost(parent_ind, current_ind);
                                cur_cost = obj.state_space_(i, j, k).cost + delta_cost;
                                if cur_cost < obj.state_space_(i + 1, m, n).cost
                                    obj.state_space_(i + 1, m, n) = struct('cost', cur_cost, 'current_s', s, 'parent_s_ind', j, 'parent_l_ind', k);
                                end
                            end
                        end
                        
                    end
                end
            end
            
            %% find the least cost in final layer
            min_cost = obj.kInf;
            min_s_ind = 1;
            min_l_ind = 1;
            for i = 1 : 1 : obj.NS
                for j = 1 : 1 : obj.NL
                    cost = obj.state_space_(obj.NT, i, j).cost;
                    if cost < min_cost
                        min_cost = cost;
                        min_s_ind = i;
                        min_l_ind = j;
                    end
                end
            end
            
            temp_waypoints = [];
            for i = obj.NT : -1 : 1
                state = obj.state_space_(i, min_s_ind, min_l_ind);
                waypoint.state_index = struct('t', i, 's', min_s_ind, 'l', min_l_ind);
                waypoint.state = state;
                temp_waypoints = [temp_waypoints, waypoint];
                min_s_ind = state.parent_s_ind;
                min_l_ind = state.parent_l_ind;
            end
            
            waypoints = temp_waypoints(obj.NT : -1 : 1);
            
            path.x = zeros(obj.config_.nfe, 1);
            path.y = zeros(obj.config_.nfe, 1);
            path.theta = zeros(obj.config_.nfe, 1);
            
            last_s = obj.start_state_.s;
            last_l = obj.start_state_.l;
            
            %% update x, y, theta
            for i = 1 : 1 : obj.NT
                parent_s = obj.start_state_.s;
                if i > 1 + obj.kEpsilon
                    parent_s = waypoints(i - 1).state.current_s;
                end
                segment = obj.InterpolateLinearly(parent_s, waypoints(i).state.parent_l_ind, waypoints(i).state_index.s, waypoints(i).state_index.l);
                
                for j = 1 : 1 : obj.nseg_
                    dl = segment(j).l - last_l;
                    ds = max(segment(j).s - last_s, obj.kEpsilon);
                    last_s = segment(j).s;
                    last_l = segment(j).l;
                    
                    [x, y] = obj.env_.GetCartesian(last_s, last_l);
                    ref_pt = obj.env_.EvaluatePointByS(last_s);
                    
                    n = (i - 1) * obj.nseg_ + j;
                    path.x(n, 1) = x;
                    path.y(n, 1) = y;
                    path.theta(n, 1) = Math.NormalizeAngle(ref_pt.theta + atan((dl / ds) / (1 - ref_pt.kappa * last_l)));
                end
            end
        end
        
        function [s, cost] = GetCost(obj, parent_ind, cur_ind)
            parent_s = obj.start_state_.s;
            parent_l = obj.start_state_.l;
            grandparent_s = obj.start_state_.s;
            grandparent_l = obj.start_state_.l;
            
            if parent_ind.t >= 1
                parent_state = obj.state_space_(parent_ind.t, parent_ind.s, parent_ind.l);
                grandparent_s_ind = parent_state.parent_s_ind;
                grandparent_l_ind = parent_state.parent_l_ind;
                parent_s = parent_state.current_s;
                parent_l = obj.GetLateralOffset(parent_ind.l);
                
                if parent_ind.t >= 2
                    grandparent_state = obj.state_space_(parent_ind.t - 1, grandparent_s_ind, grandparent_l_ind);
                    grandparent_s = grandparent_state.current_s;
                    grandparent_l = obj.GetLateralOffset(grandparent_l_ind);
                end
            end
            cur_s = parent_s + obj.station_array_(cur_ind.s);
            s = cur_s;
            if cur_s > obj.station_max_
                cost = obj.config_.weight_obstacle;
                return;
            end
            cur_l = obj.GetLateralOffset(cur_ind.l);
            ds1 = cur_s - parent_s;
            dl1 = cur_l - parent_l;
            
            ds0 = parent_s - grandparent_s;
            dl0 = parent_l - grandparent_l;
            
            cost_lateral = abs(cur_l);
            cost_lateral_change = abs(parent_l - cur_l) / (obj.station_array_(cur_ind.s) + obj.kEpsilon);
            cost_lateral_change_t = abs(dl1 - dl0) / obj.unit_time_;
            cost_longitudinal_velocity = abs(ds1 / obj.unit_time_ - obj.config_.nominal_velocity);
            cost_longitudinal_velocity_change = abs(ds1 - ds0) / obj.unit_time_;
            
            if cost_lateral_change > obj.config_.max_l_prime || ...
                    cost_lateral_change_t > obj.config_.max_l_dot || ...
                    cost_longitudinal_velocity_change > obj.config_.max_longitudinal_velocity_change
                cost = obj.config_.weight_obstacle;
                return;
            end
            
            cost_obstacle = obj.GetCollisionCost(parent_ind, cur_ind);
            if cost_obstacle >= obj.config_.weight_obstacle - obj.kEpsilon
                cost = obj.config_.weight_obstacle;
                return;
            end
            
            cost = obj.config_.weight_lateral * cost_lateral + ...
                obj.config_.weight_lateral_change * cost_lateral_change + ...
                obj.config_.weight_lateral_velocity_change *  cost_lateral_change_t + ...
                obj.config_.weight_longitudinal_velocity_bias * cost_longitudinal_velocity + ...
                obj.config_.weight_longitudinal_velocity_change * cost_longitudinal_velocity_change;
        end
        
        function cost = GetCollisionCost(obj, parent_ind, cur_ind)
            parent_s = obj.start_state_.s;
            grandparent_s = obj.start_state_.s;
            last_s = obj.start_state_.s;
            last_l = obj.start_state_.l;
            
            if parent_ind.t >= 1
                parent_state = obj.state_space_(parent_ind.t, parent_ind.s, parent_ind.l);
                parent_s = parent_state.current_s;
                
                if parent_ind.t >= 2
                    grandparent_state = obj.state_space_(parent_ind.t - 1, parent_state.parent_s_ind, parent_state.parent_l_ind);
                    grandparent_s = grandparent_state.current_s;
                end
                
                prev_path = obj.InterpolateLinearly(grandparent_s, parent_state.parent_l_ind, parent_ind.s, parent_ind.l);
                last_s = prev_path(end).s;
                last_l = prev_path(end).l;
            end
            
            path = obj.InterpolateLinearly(parent_s, parent_ind.l, cur_ind.s, cur_ind.l);
            for i = 1 : 1 : length(path)
                pt = path(i);
                dl = pt.l - last_l;
                ds = max(obj.kEpsilon, pt.s - last_s);
                last_s = pt.s;
                last_l = pt.l;
                
                [veh_x, veh_y] = obj.env_.GetCartesian(pt.s, pt.l);
                road_width = obj.env_.GetRoadWidth();
                lb = -road_width / 2.0 + obj.safe_margin_;
                ub = road_width / 2.0 - obj.safe_margin_;
                if pt.l < lb - obj.kEpsilon || pt.l > ub + obj.kEpsilon
                    cost = obj.config_.weight_obstacle + obj.kEpsilon;
                    return;
                end
                
                ref_pt = obj.env_.EvaluatePointByS(pt.s);
                veh_heading = Math.NormalizeAngle(ref_pt.theta + atan((dl / ds) / (1 - ref_pt.kappa * pt.l)));
                
                parent_time = 0.0;
                if parent_ind.t > 0
                    parent_time = obj.time_array_(parent_ind.t);
                end
                time = parent_time + (i - 1) * (obj.unit_time_ / obj.nseg_);
                
                if obj.env_.CheckVehicleDiscCollision(time, veh_x, veh_y, veh_heading)
                    cost = obj.config_.weight_obstacle + obj.kEpsilon;
                    return;
                end
            end
            cost = 0.0;
        end
        
        function path = InterpolateLinearly(obj, parent_s, parent_l_ind, cur_s_ind, cur_l_ind)
            path = repmat(struct('s', 0, 'l', 0), 1, obj.nseg_);
            
            p_s = obj.start_state_.s;
            p_l = obj.start_state_.l;
            
            if parent_l_ind >= 1
                p_s = parent_s;
                p_l = obj.GetLateralOffset(parent_l_ind);
            end
            
            cur_l = obj.GetLateralOffset(cur_l_ind);
            
            s_step = obj.station_array_(cur_s_ind) / obj.nseg_;
            l_step = (cur_l - p_l) / obj.nseg_;
            
            for i = 1 : 1 : obj.nseg_
                path(1, i).s = p_s + (i - 1) * s_step;
                path(1, i).l = p_l + (i - 1) * l_step;
            end
        end
        
        function offset = GetLateralOffset(obj, l_ind)
            road_width = obj.env_.GetRoadWidth();
            lb = -road_width / 2.0 + obj.safe_margin_;
            ub = road_width / 2.0 - obj.safe_margin_;
            offset = lb + (ub - lb) * obj.lateral_array_(l_ind);
        end
    end
end


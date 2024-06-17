classdef LIOCPPlanner < handle
    
    properties
        dp_config_;
        config_;
        vehicle_param_;
        start_state_;
        
        safe_corridor_;
        env_;
        
        iteractive_solver_;
        infeasibility_evaluator_;
    end
    
    methods
        function obj = LIOCPPlanner(start_state, env)
            obj.start_state_ = start_state;
            obj.dp_config_ = GetDPPlannerConfig();
            obj.config_ = GetLIOCPPlannerConfig();
            obj.vehicle_param_ = GetVehicleParams();
            obj.safe_corridor_ = SafeCorridor(env);
            obj.BUildIterativeNLP();
            obj.env_ = env;
        end
        
        function obj = BUildIterativeNLP(obj)
            import casadi.*
            tf = obj.dp_config_.tf;
            n = obj.dp_config_.nfe;
            dt = tf/ (n - 1);
            front_length = 0.75 * obj.vehicle_param_.length - obj.vehicle_param_.rear_oh;
            rear_length = 0.25 * obj.vehicle_param_.length - obj.vehicle_param_.rear_oh;
            
            x = SX.sym('x', n);
            y = SX.sym('y', n);
            theta = SX.sym('theta', n);
            v = SX.sym('v', n);
            delta = SX.sym('detla', n);
            a = SX.sym('a', n);
            omega = SX.sym('omega', n);
            jerk = SX.sym('jerk', n);
            
            xf = SX.sym('xf', n);
            yf = SX.sym('yf', n);
            xr = SX.sym('xr', n);
            yr = SX.sym('yr', n);
            
            p_weight_penalty = SX.sym('weight_penalty');
            p_ref_x = SX.sym('ref_x', n, 1);
            p_ref_y = SX.sym('ref_y', n, 1);
            p_ref_theta = SX.sym('ref_theta', n, 1);
            
            g_x_kin = x(2 : end) - (x(1 : end - 1) + dt * v(1 : end - 1) .* cos(theta(1 : end - 1)));
            g_y_kin = y(2 : end) - (y(1 : end - 1) + dt * v(1 : end - 1) .* sin(theta(1 : end - 1)));
            g_theta_kin = theta(2 : end) - (theta(1 : end - 1) + dt * v(1 : end - 1) .* tan(delta(1 : end - 1)) / obj.vehicle_param_.wheel_base);
            g_v_kin = v(2 : end) - (v(1 : end - 1) + dt * a(1 : end - 1));
            g_delta_kin = delta(2 : end) - (delta(1 : end - 1) + dt * omega(1 : end - 1));
            g_a_kin = a(2 : end) - (a(1 : end - 1) + dt * jerk(1 : end - 1));
            
            g_xf_kin = xf - (x + front_length * cos(theta));
            g_yf_kin = yf - (y + front_length * sin(theta));
            g_xr_kin = xr - (x + rear_length * sin(theta));
            g_yr_kin = yr - (y + rear_length * cos(theta));
            infeasibility = sumsqr([g_x_kin; g_y_kin; g_theta_kin; g_v_kin; g_delta_kin; g_a_kin; g_xf_kin; g_yf_kin; g_xr_kin; g_yr_kin]);
            
            f_obj = obj.config_.weight_x * sumsqr(x - p_ref_x) + obj.config_.weight_y * sumsqr(y - p_ref_y) + obj.config_.weight_theta * sumsqr(theta - p_ref_theta) + ...
                obj.config_.weight_u * (sumsqr(jerk) + obj.config_.weight_omega * sumsqr(omega)) + p_weight_penalty * infeasibility;
            p = [p_weight_penalty; p_ref_x; p_ref_y; p_ref_theta];
            opt_x = [x; y; theta; v; delta; a; omega; jerk; xf; yf; xr; yr];
            nlp = struct('x', opt_x, 'f', f_obj, 'p', p);
            options = obj.SetNLPOption();
            obj.iteractive_solver_ = nlpsol('iteractive_solver', 'ipopt', nlp, options);
            obj.infeasibility_evaluator_ = Function('inf', {opt_x}, {infeasibility});
        end
        
        function options = SetNLPOption(obj)
            options = struct;
            options.print_time = 0;
            options.ipopt.max_iter = 100;
            options.ipopt.print_level = 0;
            options.ipopt.acceptable_tol = 1e-8;
            options.ipopt.acceptable_obj_change_tol = 1e-6;
            options.ipopt.linear_solver = 'mumps';
        end
        
        function [infeasibility, traj] = SolveIteractively(obj, weight_penalty, constraints, guess, reference)
            identity = ones(obj.dp_config_.nfe, 1);
            
            lb_x = min([constraints.f_min_x; constraints.r_min_x]) * identity;
            ub_x = max([constraints.f_max_x; constraints.r_max_x]) * identity;
            lb_y = min([constraints.f_min_y; constraints.r_min_y]) * identity;
            ub_y = max([constraints.f_max_y; constraints.r_max_y]) * identity;
            lb_theta = -inf * identity;
            ub_theta = inf * identity;
            lb_v = 0.0 * identity;
            ub_v = obj.vehicle_param_.max_velocity * identity;
            lb_delta = -obj.vehicle_param_.max_delta * identity;
            ub_delta = obj.vehicle_param_.max_delta * identity;
            lb_a = -obj.vehicle_param_.max_acceleration * identity;
            ub_a = obj.vehicle_param_.max_acceleration * identity;
            lb_omega = -obj.vehicle_param_.max_omega * identity;
            ub_omega = obj.vehicle_param_.max_omega * identity;
            lb_jerk = -obj.vehicle_param_.max_jerk * identity;
            ub_jerk = obj.vehicle_param_.max_jerk * identity;

            lb_x(1) = obj.start_state_.x;
            ub_x(1) = obj.start_state_.x;
            lb_y(1) = obj.start_state_.y;
            ub_y(1) = obj.start_state_.y;
            lb_theta(1) = obj.start_state_.theta;
            ub_theta(1) = obj.start_state_.theta;
            lb_v(1) = obj.start_state_.v;
            ub_v(1) = obj.start_state_.v;
            lb_delta(1) = obj.start_state_.delta;
            ub_delta(1) = obj.start_state_.delta;
            lb_a(1) = obj.start_state_.a;
            ub_a(1) = obj.start_state_.a;
            lb_omega(1) = obj.start_state_.omega;
            ub_omega(1) = obj.start_state_.omega;
            lb_jerk(1) = obj.start_state_.jerk;
            ub_jerk(1) = obj.start_state_.jerk;
            
            lb_xf = constraints.f_min_x;
            lb_yf = constraints.f_min_y;
            lb_xr = constraints.r_min_x;
            lb_yr = constraints.r_min_y;
            ub_xf = constraints.f_max_x;
            ub_yf = constraints.f_max_y;
            ub_xr = constraints.r_max_x;
            ub_yr = constraints.r_max_y;
            
            lbx = [lb_x; lb_y; lb_theta; lb_v; lb_delta; lb_a; lb_omega; lb_jerk; lb_xf; lb_yf; lb_xr; lb_yr];
            ubx = [ub_x; ub_y; ub_theta; ub_v; ub_delta; ub_a; ub_omega; ub_jerk; ub_xf; ub_yf; ub_xr; ub_yr];
            
            ref_x = reference.x;
            ref_y = reference.y;
            ref_theta = reference.theta;
            
            p = [weight_penalty; ref_x; ref_y; ref_theta];
            res = obj.iteractive_solver_('x0', guess, 'lbx', lbx, 'ubx', ubx, 'p', p);
            
            traj = full(res.x);
            infeasibility = full(obj.infeasibility_evaluator_(traj));
        end
        
        function [flag, traj] = Plan(obj, coarse)
            guess = obj.CalculateInitialGuess(coarse);
            iter = 0;
            weight_penalty = obj.config_.weight_penalty_init;
            
%             figure;
%             [~, constraints] = obj.safe_corridor_.FormulateConstraints(guess);
%             traj_plot = obj.DataTransform(guess);
%             Plot.PlotSafeCorridor(obj.env_, traj_plot, constraints);
            
            while iter < obj.config_.max_iter
                [~, constraints] = obj.safe_corridor_.FormulateConstraints(guess);
                [infeasibility, guess] = obj.SolveIteractively(weight_penalty, constraints, guess, coarse);
                
                figure;
                title_string = strcat(num2str(iter + 1), 'th iter safe corridor'); 
                traj_plot = obj.DataTransform(guess);
                Plot.PlotSafeCorridor(obj.env_, traj_plot, constraints, title_string);
                
                if infeasibility < obj.config_.varepsilon_tol
                    traj = obj.DataTransform(guess);
                    flag = true;
                    disp('convergence');
                    return;
                else
                    weight_penalty = obj.config_.alpha * weight_penalty;
                end
                iter = iter + 1;
            end
            disp('reach max iter');
            flag = false;
            traj = obj.DataTransform(guess);
        end
        
        function guess = CalculateInitialGuess(obj, coarse)
            n = obj.dp_config_.nfe;
            dt = obj.dp_config_.tf / (n - 1);
            guess = zeros(12 * n, 1);
            %% updata x, y theta
            guess(1: n, 1) = coarse.x;
            guess(n + 1: 2 * n, 1) = coarse.y;
            guess(2 * n + 1: 3 * n, 1) = coarse.theta;
            
            %% update start point
            guess(1, 1) = obj.start_state_.x;
            guess(n + 1, 1) = obj.start_state_.y;
            guess(2 * n + 1, 1) = obj.start_state_.theta;
            guess(3 * n + 1, 1) = obj.start_state_.v;
            guess(4 * n + 1, 1) = obj.start_state_.delta;
            guess(5 * n + 1, 1) = obj.start_state_.a;
            guess(6 * n + 1, 1) = obj.start_state_.omega;
            guess(7 * n + 1, 1) = obj.start_state_.jerk;
            
            %% updata velocity, delta
            guess(3 * n + 2 : 4 * n, 1) = ...
                min(obj.vehicle_param_.max_velocity , ...
                hypot(guess(2 : n, 1) - guess(1 : n - 1, 1), ...
                guess(n + 2 : 2 * n, 1) - guess(n + 1 : 2 * n - 1, 1)) / dt);
            guess(4 * n + 2 : 5 * obj.dp_config_.nfe, 1) = ...
                min(obj.vehicle_param_.max_delta, max(-obj.vehicle_param_.max_delta, ...
                atan((guess(2 * n + 2 : 3 * n, 1) - ...
                guess(2 * n + 1 : 3 * n - 1, 1)) ...
                * obj.vehicle_param_.wheel_base ./ (guess(3 * n + 1 : 4 * n - 1, 1) * dt))));
            
            %% update acceleration, omega
            guess(5 * n + 2 : 6 * n, 1) = ...
                min(obj.vehicle_param_.max_acceleration, max(-obj.vehicle_param_.max_acceleration, ...
                (guess(3 * n + 2 : 4 * n, 1) - ...
                guess(3 * n + 1 : 4 * n - 1, 1)) / dt));
            
            guess(6 * n + 2 : 7 * n, 1) = ...
                min(obj.vehicle_param_.max_acceleration, max(-obj.vehicle_param_.max_acceleration, ...
                (guess(4 * n + 2 : 5 * n, 1) - guess(4 * n + 1 : 5 * n - 1, 1)) / dt));
            
            %% update jerk
            guess(7 * n + 2 : 8 * n, 1) = ...
                min(obj.vehicle_param_.max_jerk, max(-obj.vehicle_param_.max_jerk, ...
                (guess(5 * n + 2 : 6 * n, 1) - guess(5 * n + 1 : 6 * n - 1, 1)) / dt));
            
            %% updata xf, yf, xr, yr
            front_length = 0.75 * obj.vehicle_param_.length - obj.vehicle_param_.rear_oh;
            rear_length = 0.25 * obj.vehicle_param_.length - obj.vehicle_param_.rear_oh;
            guess(8 * n + 1 : 9 * n, 1) = ...
                guess(1 : n, 1) + front_length * cos(guess(2 * n + 1 : 3 * n, 1));
            guess(9 * n + 1 : 10 * n, 1) = ...
                guess(n + 1 : 2 * n, 1) + front_length * sin(guess(2 * n + 1 : 3 * n, 1));
            guess(10 * n + 1 : 11 * n, 1) = ...
                guess(1 : n, 1) + rear_length * cos(guess(2 * n + 1 : 3 * n, 1));
            guess(11 * n + 1 : 12 * n, 1) = ...
                guess(n + 1 : 2 * n, 1) + rear_length * sin(guess(2 * n + 1 : 3 * n, 1));
        end
        
        function trajectory = DataTransform(obj, traj)
            n = obj.dp_config_.nfe;
            dt = obj.dp_config_.tf / (n - 1);
            trajectory.t = dt * [0 : 1 : n - 1]';
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


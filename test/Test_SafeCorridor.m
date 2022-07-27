clc; clear all; close all;

scenarios = Scenario();

for scenario = scenarios
    start_pt = scenario.lane.center_line(1);
    env = Environment(scenario);
    start_state = struct('x', 0, 'y', 0, 'theta', 0, 'v', 0, 'a', 0, 'jerk', 0, 'delta', 0, 'omega', 0);
    dp_planner = DpPlanner(env);
    path = dp_planner.Plan(start_state);
    
    guess = Utils.CalculateInitialGuess(path, start_state);
    
    safe_corridor = SafeCorridor(env);
    [flag, constraints] = safe_corridor.FormulateConstraints(guess);
    traj = Utils.DataTransform(guess);
    if flag
        Plot.PlotSafeCorridor(env, traj, constraints);
    end
    break;
end
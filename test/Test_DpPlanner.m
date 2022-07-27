clc; clear all; close all;

scenarios = Scenario();

for scenario = scenarios
    start_pt = scenario.lane.center_line(1);
    env = Environment(scenario);
    start_state = struct('x', 0, 'y', 0, 'theta', 0, 'velocity', 0, 'acceleration', 0, 'jerk', 0, 'delta', 0, 'omega', 0);
    dp_planner = DpPlanner(env);
    states = dp_planner.Plan(start_state);
    plot(states.x, states.y);
    break;
end
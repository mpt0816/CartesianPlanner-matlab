clc; clear all; close all;

%% 添加路径
global current_folder
current_folder = pwd;
addpath(genpath(current_folder));

%% 生成测试场景
scenarios = Scenario();

for scenario = scenarios
    start_pt = scenario.lane.center_line(1);
    env = Environment(scenario);
    start_state = struct('x', env.GetReferenceLine().x(1), 'y', env.GetReferenceLine().y(1), 'theta', env.GetReferenceLine().theta(1), 'v', 0, 'delta', 0, 'a', 0, 'omega', 0, 'jerk', 0);
    dp_planner = DpPlanner(env);
    path = dp_planner.Plan(start_state);
    
    planner = LIOCPPlanner(start_state, env);
    [flag, traj] = planner.Plan(path);
    
    Plot.PlotResult(env, traj)
end
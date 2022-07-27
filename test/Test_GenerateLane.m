clc; clear all; close all;

%% 生成测试场景
scenarios = Scenario();

for scenario = scenarios
    figure;
    Plot.PlotLane(scenario.lane);
end

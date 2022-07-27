clc; clear all; close all;

scenarios = Scenario();

for scenario = scenarios
    Plot.PlotDynamicObstacles(scenario.dynamic_obstacles);
    break;
end




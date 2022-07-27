function scenarios = Scenario()
scenarios = [];

%% scenario 2: 直道，有障碍物
scenario.name = 'scenario#2#straight-lane';
key_points = KeyPointStraightPath();
lane_generator = GenerateLane();
lane_generator.SetLaneWidth(5);
lane_generator.SetShoulderHeight(0.1);
scenario.lane = lane_generator.GenerateLaneWithPointList(key_points);
scenario.key_points = key_points;

obstacle_type = 'bus';
%     obstacle_type = 'car';
num_of_obstacle = 2;
offset_center = 1.2;
obstacle_generator = GenerateStaticObstacle(scenario.lane, obstacle_type, num_of_obstacle, offset_center);
scenario.static_obstacles = obstacle_generator.GetObstacles();

obstacle_type = 'bus';
%     obstacle_type = 'car';
offset_center = 3.0;
start_s = 50.0;
start_v = 10.0;
start_a = -1.2;
dynamic_obstacle_generator = GenerateDynamicObstacle(scenario.lane);
dynamic_obstacle_generator.AddAlignedObstacle(obstacle_type, start_s, start_v, start_a, offset_center);
scenario.dynamic_obstacles = dynamic_obstacle_generator.GetObstacles();

scenarios = [scenarios, scenario];

end
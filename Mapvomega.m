clc;
close all;
clear;
rng('default');
map = mapClutter(24,{'Box','Circle'},'MapResolution',15);
planningMap = copy(map); % This line assumes there's a way to copy or clone the map object.
inflate(planningMap, 0.8); % Inflate obstacles in the planning map.

%Use the map to create a plannerAStarGrid object.

planner = plannerAStarGrid(planningMap, 'GCost', 'Euclidean');

%Define the start and goal points.
start = [3 3];
goal = [49 45];
%Plan a path from the start point to the goal point.
[path,Pathinfo] = plan(planner,start,goal,'world');
 %Check if a path was found
if path
    % Path contains the path from start to goal
    % You can now use or store foundPath for future use
    % For example, store in a variable
    fprintf('Path Cost: %f\n', Pathinfo.PathCost);
    fprintf('Number of Nodes Explored: %f\n', Pathinfo.NumNodesExplored);

%path generated by A* algorithm which will be used as our referrence trajectory    
x=path(:,1);
y=path(:,2);
theta=atan(y/x);
theta=theta(:,750);
else
    disp('No path found.');
end

Sim_3_MPC_Robot_PS_obs_avoid_mul_sh(x,y,theta,map);


addpath(genpath('..\toolbox\helperOC\'));
addpath(genpath('..\toolbox\ToolboxLS\'));

%% TODO
% Define the grid for the computation: g
% g =...
grid_min = [-4; -4]; % Lower corner of computation domain
grid_max = [4; 4];    % Upper corner of computation domain
N = [80; 80];         % Number of grid points per dimension
g = createGrid(grid_min, grid_max, N);

%% TODO Hint: look into shapeSphere() function 
% Define the failure set: data0
% data0 = ....
R = 1;
data0 = shapeSphere(g,[0;0],R);

%% time vector
t0 = 0;
tMax = 2;
dt = 0.05;
tau = t0:dt:tMax;

%% problem parameters

% input bounds
speed = 1;
theta_max = pi/6;
dist_max = 0;

% control trying to min or max value function?
uMode = 'max';
dMode = 'min';

%% Pack problem parameters

% Define dynamic system
drone = drone_2d([0, 0], theta_max, speed, dist_max, [1, 2]);

% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = drone;
schemeData.accuracy = 'high'; %set accuracy
schemeData.uMode = uMode;
schemeData.dMode = dMode;

%% Compute value function
% HJIextraArgs.visualize = true; %show plot
HJIextraArgs.visualize.valueSet = 1;
HJIextraArgs.visualize.initialValueSet = 1;
HJIextraArgs.visualize.figNum = 1; %set figure number
HJIextraArgs.visualize.deleteLastPlot = false; %delete previous plot as you update

% uncomment if you want to see a 2D slice
% HJIextraArgs.visualize.plotData.plotDims = [1 1]; %plot x, y
HJIextraArgs.visualize.viewAngle = [0,90]; % view 2D

%[data, tau, extraOuts] = ...
% HJIPDE_solve(data0, tau, schemeData, minWith, extraArgs)
[data, tau2, ~] = ...
  HJIPDE_solve(data0, tau, schemeData, 'zero', HJIextraArgs);
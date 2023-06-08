addpath(genpath('..\toolbox\helperOC\'));
addpath(genpath('..\toolbox\ToolboxLS\'));
addpath(genpath('..\mod_hj_pde_solver\'));

%% TODO
% Define the grid for the computation: g
% g =...
grid_min = -4; % Lower corner of computation domain
grid_max = 4;    % Upper corner of computation domain
N = 80;         % Number of grid points per dimension
g = createGrid(grid_min, grid_max, N); % generate a 1d grid
dx_grid = (grid_max - grid_min)/N

%% Define the targetset on 1d GRID
% data0 = ....
R = 0.4;
R_v = 3;
data0 = zeros(N,1); % 80*1 vec
data0(floor((R_v-grid_min)/dx_grid):floor((R_v+R-grid_min)/dx_grid)) = -0.5; % any neg number

%% time vector
t0 = 0;
tMax = 5; % compute roa for 5 secs
dt = 0.05;
tau = t0:dt:tMax;

%% problem parameters

% input bounds
speed_range = 0.25;
disturbance_range = 0;

uMode = 'min';
dMode = 'max';

x_init = 0; % doesnt matter for brt calc
x_dim = 1; % if 2d sys, [1,2]
obst_range = [-1,0];

%% Pack problem parameters

% Define dynamic system

car_1d = car_1d_hybrid(x_init, speed_range, disturbance_range, obst_range, x_dim);

% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = car_1d;
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
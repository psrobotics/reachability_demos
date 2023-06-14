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
% R = 1;
% R_v = 3;
% data0 = zeros(N,1); % 80*1 vec
% data0(floor((R_v-grid_min)/dx_grid):floor((R_v+R-grid_min)/dx_grid)) = -1; % any neg number
%%
%% target set
R = 0.5;
% data0 = shapeCylinder(grid,ignoreDims,center,radius)
data0 = shapeCylinder(g, [], [3.5], R); % smooth target set

%% time vector
t0 = 0;
tMax = 6; % compute roa
dt = 0.05;
tau = t0:dt:tMax;
t_step_num = tMax/dt

%% problem parameters

% input bounds
speed_range = 0.25;
disturbance_range = 0;

uMode = 'min';
dMode = 'max';

x_init = 0; % doesnt matter for brt calc
x_dim = 1; % if 2d sys, [1,2]
obst_range = [-1,0];
q_mode_num = 2;

%% Pack problem parameters

% Define dynamic system

car_1d = car_1d_hybrid(x_init, speed_range, disturbance_range, q_mode_num, obst_range, x_dim);
%car_1d = Dog1D(x_init,speed_range);

% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = car_1d;
schemeData.accuracy = 'high'; %set accuracy
schemeData.uMode = uMode;
schemeData.dMode = dMode;
% add an empty arr to store selected q mode, changed
schemeData.q_mode_arr = zeros(80,t_step_num);
schemeData.q_index = 1;


%% Compute value function
% HJIextraArgs.visualize = true; %show plot
HJIextraArgs.visualize.valueSet = 1;
HJIextraArgs.visualize.initialValueSet = 1;
HJIextraArgs.visualize.figNum = 1; %set figure number
HJIextraArgs.visualize.deleteLastPlot = false; %delete previous plot as you update

% uncomment if you want to see a 2D slice
% HJIextraArgs.visualize.plotData.plotDims = [1 1]; %plot x, y
HJIextraArgs.visualize.viewAngle = [0,90]; % view 2D
% set axis limits

%[data, tau, extraOuts] = ...
% HJIPDE_solve(data0, tau, schemeData, minWith, extraArgs)
[data, tau2, ~] = ...
  HJIPDE_solve_hybrid(data0, tau, schemeData, 'zero', HJIextraArgs);
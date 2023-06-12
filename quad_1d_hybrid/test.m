clc; clear;
close all; warning off;
%%
addpath(genpath('..\toolbox\helperOC\'));
addpath(genpath('..\toolbox\ToolboxLS\'));
addpath(genpath('..\mod_hj_pde_solver\'));
addpath(genpath('quad_mpc\'));

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
TAR_x = 3.5;
% data0 = shapeCylinder(grid,ignoreDims,center,radius)
data0 = shapeCylinder(g, [], [TAR_x, 0], R); % smooth target set

%% time vector
t0 = 0;
tMax = 10; % compute roa
dt = 0.05;
tau = t0:dt:tMax;
t_step_num = tMax/dt

%% problem parameters

% input bounds
speed_range = 0.25;
disturbance_range = 0;

uMode = 'min';
dMode = 'max';

x_init = -3; % doesnt matter for brt calc
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
[data, tau2, extra_outs, schemeData] = ...
  HJIPDE_solve_hybrid(data0, tau, schemeData, 'zero', HJIextraArgs);
%% unpack data
returned_q_mode_arr = schemeData.innerData.q_mode_arr(:,end);

%% sim traj
% sim opti traj of the quad
x_arr = x_init;
t_arr = [];
ctr_arr = [];
brs_t_ind_arr = [];
dt = tau(2)-tau(1);
q_mode_arr = [];

for t_n = tau
    t_arr = [t_arr, t_n];
    x_n = x_arr(:,end);
    
    % check if reaches target
    if x_n > TAR_x
        break;
    end

    % get current brs
    % determine the earliest time that the current state is in the reachable set
    t_earliest_i = ceil(tau(end)/dt);
    for t_i = 1:t_earliest_i
        value_at_x = eval_u(g, data(:, t_i), x_n);
        if value_at_x<0
            t_earliest_i = t_i;
            break;
        end
    end
    brs_t_ind_arr = [brs_t_ind_arr, t_earliest_i];
    brs_at_t = data(:,t_earliest_i);
    deriv_t = computeGradients(g, brs_at_t);
    deriv_n = eval_u(g, deriv_t, x_n);
    % get optimal control
    opti_ctr_t = car_1d.optCtrl([],[],deriv_n,'min');
    ctr_arr = [ctr_arr, opti_ctr_t];
    % get current q mode
    q_mode_index = ceil((x_n - grid_min)/dx_grid);
    q_mode_n = returned_q_mode_arr(q_mode_index);
    q_mode_arr=[q_mode_arr,q_mode_n];
    % sim next step
    [~, x_next_arr] = ode113(@(t,x) car_1d.dynamics(t, x, opti_ctr_t, 0, q_mode_n), [0 dt], x_n);
    x_next = x_next_arr(end,:);
    x_arr = [x_arr, x_next];

    fprintf('simulating timestep %f\n', t_n);
end

%% quad mpc sim
[world_params, body_params, ctr_params, path] = hardware_params();

addpath(path.casadi);
import casadi.*;

%% Get dynamics
[dyn_f] = get_srb_dynamics(world_params, body_params, path);

%%
ctr_params.x_init_tar_val = [0; 0; 0; x_init; 0; ctr_params.init_z]; % init state
ctr_params.dx_init_tar_val = [0; 0; 0; 1; 0; 0]; % init d state
ctr_params.x_final_tar_val = [0; 0; 0; TAR_x; 0; ctr_params.init_z]; % final target state r p y; x y z
ctr_params.dx_final_tar_val = [0; 0; 0; 1; 0; 0];

ctr_params.normal_height = 0.33;
ctr_params.obst_height = 0.18;

%% Form the mpc problem
[mpc_v, mpc_c, mpc_p] = form_mpc_prob(world_params, body_params, ctr_params, dyn_f, path);

[boundray_v] = add_state_boundaries(mpc_v, mpc_c, world_params, body_params, ctr_params, path);
%% modify desired body traj based on brt sim result, use tar x velosity and desied body height
[ref_traj_v] = fpp_planner_local(world_params, body_params, ctr_params, path, x_arr, q_mode_arr);

%% Slove the NLP prob
sol = mpc_p.solver('x0',ref_traj_v.x0,...
                   'lbx',boundray_v.lbx,...
                   'ubx',boundray_v.ubx,...
                   'lbg',boundray_v.lbg,...
                   'ubg',boundray_v.ubg,...
                   'p',ref_traj_v.p);
               
  %% Unpack data
  [x_sol, f_sol, fp_l_sol, fp_g_sol] = unpacks_sol(sol, body_params, ctr_params, path);
  
  %% we only sim until it reaches target
  brt_sim_i_len = length(q_mode_arr);
  rbt_anime(x_sol,f_sol,fp_g_sol,[],brt_sim_i_len*dt,brt_sim_i_len);
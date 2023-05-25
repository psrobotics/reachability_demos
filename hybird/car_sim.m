clear all
close all

addpath(genpath('D:\matlab_lib\helperOC\'));
addpath(genpath('D:\matlab_lib\ToolboxLS\'));

%% load data
alpha = 1.0;
% alpha = 1.0;

data_file_str = strcat('data_with_reset_map_alpha_', num2str(100*alpha));
data_file_str = strcat(data_file_str, '_t_3');
load(data_file_str);
params.alpha = alpha;

%% init states, car system
x0_original = [6; 1; -pi/2];
params.dt = tau(2)-tau(1);

dubins_car = hybird_mod_dubins_car([], x0_original, params, 'parametrized'); % select dyn model

%% sim process
params.tar_pos = [0,4];
params.tar_r = 0.5;

[x_arr_tmp, ctr_arr_tmp, t_arr_tmp, brs_t_ind_arr] =...
    car_sim_single_traj(dubins_car, x0_original, grid, data, tau, params);

%% vis process
car_anime(x_arr_tmp, ctr_arr_tmp, t_arr_tmp, brs_t_ind_arr, grid, data, params);

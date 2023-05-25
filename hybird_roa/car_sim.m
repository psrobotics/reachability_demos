clear all
close all

addpath(genpath('D:\matlab_lib\helperOC\'));
addpath(genpath('D:\matlab_lib\ToolboxLS\'));
addpath(genpath('data\'));
addpath(genpath('visualize\'));
addpath(genpath('hj_pde\'));

%% load data
alpha = 1.0;
% alpha = 1.0;

data_file_str = strcat('data_with_reset_map_alpha_', num2str(100*alpha));
data_file_str = strcat(data_file_str, '_t_6');
load(data_file_str);
params.alpha = alpha;

%% init states, car system
params.dt = tau(2)-tau(1);
params.tar_pos = [0,4];
params.tar_r = 0.5;

%% create a set of random init states
test_case_num = 20;
x0_arr = zeros(3,test_case_num);
x0_arr(1,:) = (7.5-(-7.5)).*rand(test_case_num,1) - 7.5;
x0_arr(2,:) = (7.5-0).*rand(test_case_num,1) - 0;
x0_arr(3,:) = (pi-(-pi)).*rand(test_case_num,1) - pi;

for test_i = 1:test_case_num

    x0_original = x0_arr(:,test_i)
    dubins_car = hybird_mod_dubins_car([], x0_original, params, 'parametrized'); % select dyn model
    % sim process
    [x_arr_tmp, ctr_arr_tmp, t_arr_tmp, brs_t_ind_arr] =...
        car_sim_single_traj(dubins_car, x0_original, grid, data, tau, params);
    %% vis process
    BRT_2d_layer = get_2d_brt_vis(x_arr_tmp, ctr_arr_tmp, t_arr_tmp, brs_t_ind_arr, grid, data, params);
    %% car anime
    car_anime(x_arr_tmp, ctr_arr_tmp, t_arr_tmp, brs_t_ind_arr, grid, data, params, BRT_2d_layer,1);
end

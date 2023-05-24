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
x0_original = [4.0; 1.0; -pi/2];
t_arr = tau;
dt = t_arr(2)-t_arr(1);

dubins_car = hybird_mod_dubins_car([], x0_original, params, 'parametrized'); % select dyn model
x_arr = x0_original;

%% sim process
for t_n = t_arr
    t_n
    x_curr = x_arr(:,end)
    opti_ctr = eval_u(grid,safety_controller,x_curr')
    x_next = dubins_car.updateStateWithResetMap(opti_ctr, dt, x_curr)
    x_arr = [x_arr, x_next];
end

%%
x_curr = [0,0,0]
opti_ctr = eval_u(grid,safety_controller,x_curr)

%% vis
plot(x_arr(1,:),x_arr(2,:))
axis equal

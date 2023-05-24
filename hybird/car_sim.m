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
dt = tau(2)-tau(1);

dubins_car = hybird_mod_dubins_car([], x0_original, params, 'parametrized'); % select dyn model

x_arr = x0_original;
t_arr = 0;

%% sim process
for t_n = tau
    
    t_arr(end) = t_n;
    x_curr = x_arr(:,end);
    % get opti control
    opti_ctr = eval_u(grid,safety_controller,x_curr');
    % check if reach target set
    tar_state = check_in_target(x_curr,[0,0],0.5);
    if tar_state == 1
        break;
    end
    % sim next step
    x_next = dubins_car.updateStateWithResetMap(opti_ctr, dt, x_curr)
    x_arr = [x_arr, x_next];
end

%% vis process
for t_n = t_arr
    t_n
end
plot(x_arr(1,:),x_arr(2,:))
axis equal

%% check if the car reaches target area
function [in_target] = check_in_target(car_state, tar_pos, tar_r)
    x = car_state(1);
    y = car_state(2);
    dist = sqrt((x-tar_pos(1))^2+(y-tar_pos(2))^2) - tar_r;

    if dist<0
        in_target = 1;
    else
        in_target = 0;
    end
end
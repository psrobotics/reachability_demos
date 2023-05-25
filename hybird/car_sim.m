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
dt = tau(2)-tau(1);

dubins_car = hybird_mod_dubins_car([], x0_original, params, 'parametrized'); % select dyn model

x_arr = x0_original;
t_arr = 0;

%% sim process
params.tar_pos = [0,4];
params.tar_r = 0.5;

[x_arr_tmp, ctr_arr_tmp, t_arr_tmp] =...
    car_sim_single_traj(dubins_car, x0_original, grid, data, tau, params);

%%
for t_n = tau
    
    t_arr(end) = t_n;
    x_curr = x_arr(:,end);
    % get opti control

    % get current brs
    % Determine the earliest time that the current state is in the reachable set
    % Binary search
          % upper = floor(tau(end)/dt)
          % lower = 1 % search index range
          % valueAtX = eval_u(grid, data(:,:,:, 31), x_curr)
          % tEarliest = find_earliest_BRS_ind(grid, data, x_curr, upper, lower)
          tEarliest = floor(tau(end)/dt);
          for it = 1:floor(tau(end)/dt)
              valueAtX = eval_u(grid, data(:,:,:, it), x_curr);
              if valueAtX < 0
                  tEarliest = it;
                  break;
              end
          end

          % BRS at current time
          BRS_at_t = data(:,:,:, tEarliest);
          Deriv = computeGradients(grid, BRS_at_t);
          deriv = eval_u(grid, Deriv, x_curr);
          opti_ctr = dubins_car.optCtrl([],[],deriv,'min');


    %opti_ctr = eval_u(grid,safety_controller,x_curr);
    % check if reach target set
    tar_state = check_in_target(x_curr,[0,4],0.5);
    if tar_state == 1
        break;
    end
    % sim next step
    x_next = dubins_car.updateStateWithResetMap(opti_ctr, dt, x_curr);
    x_arr = [x_arr, x_next];
end

%% vis process
size_arr = size(x_arr);
len = size_arr(2);

%%
BRT_2d_layer = zeros(80,40);
ind = 1:80*40;
[I1, I2] = ind2sub([80,40], ind);
    for j = ind
        i1 = I1(j); % current element indector
        i2 = I2(j);
        BRT_2d_layer(i1,i2) = eval_u(grid, data(:,:,:,end), [i1*params.grid_dx1-8,i2*params.grid_dx1,-pi/2]);
    end
%%
%BRT_2d_layer = normalize(BRT_2d_layer, "range",[-1,1]);
%BRT_2d_layer = normalize(BRT_2d_layer, "norm");

%%

for k = 1:len
    clf;
    hold on;
    axis equal;
    grid on;

    x_t = x_arr(:,k);
    r_mat = rot_zyx([0,0,x_t(3)]);

    view(30,40);
    %view(90,0)
    axis([-8,8,0,8,0,1.5]);

    plot_cube(r_mat,1.1,0.6,0.2,[x_t(1);x_t(2);0.8]);

    [X,Y]=meshgrid(-8+params.grid_dx1:params.grid_dx1:8,0+params.grid_dx1:params.grid_dx1:8);
    surf(X,Y,-1*BRT_2d_layer')
    colormap summer

    pause(0.1);
    hold on;

end


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

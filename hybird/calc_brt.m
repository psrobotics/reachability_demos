clear all
close all

addpath(genpath('D:\matlab_lib\helperOC\'));
addpath(genpath('D:\matlab_lib\ToolboxLS\'));

%%
params.v = 2; % forward_vel
params.R = 2; % cycle_r
params.u_bound = 1; % turning limit

%% Load target set and grid
load('dubins_target_binary_hybrid')
% Load value function set
target_function = load_target_function_from_h5('dubins_target_hybrid_fmm.h5');

%% solver setup
schemeData.grid = grid;
schemeData.uMode = 'min'; % control trying to min the cost fcn, reachable set

schemeData.dynSys = ModifiedDubinsCar([], [], params); % select dyn model

schemeData.accuracy = 'high';
schemeData.hamFunc = @genericHam;
schemeData.partialFunc = @genericPartial;
[schemeData.dissFunc, ~, schemeData.derivFunc] = ...
    getNumericalFuncs('global', schemeData.accuracy);

HJIextraArgs.stopConverge = 1;
HJIextraArgs.targetFunction = target_function;

HJIextraArgs.visualize.valueSet = 1;
HJIextraArgs.visualize.initialValueSet = 1;
HJIextraArgs.visualize.figNum = 1; % set figure number
HJIextraArgs.visualize.deleteLastPlot = false; % delete previous plot as you update

%% sim time
t0 = 0;
dt = 0.1;
t_max = 30;
tau = t0:dt:t_max;
data0 = target_function; % pre-defined target set value function (generated from python script)

alphs = 0.7; % nonlinear reset para

%% cal brt
schemeData.reset_map = get_reset_map_parametrized(grid, params);
[data, tau, extraOuts] = ...
        HJIPDE_solve_with_reset_map(data0, tau, schemeData, 'minVWithL', HJIextraArgs);

data_file_str = strcat('data_with_reset_map_alpha_', num2str(100*alpha));
data_file_str = strcat(data_file_str, '_t_');
data_file_str = strcat(data_file_str, num2str(t_max));
save(strcat(data_file_str, '.mat'), 'grid', 'data0', 'params', 'data', 'tau'); % save data

%% tar fcn load
function target_function = load_target_function_from_h5(file_name)
fid = H5F.open(file_name);
dset_id = H5D.open(fid,'data');
target_function = permute(H5D.read(dset_id), [3, 2, 1]);
end

%% get rest map indector
function reset_map = get_reset_map_parametrized(grid, params)

    eps = 1e-5;
    R = params.R;
    N = grid.N; % grid num vector
    M = N(3) / 2;
    ind = 1:prod(N); % how many elements in grid, = N(1)*N(2)*N(3)
    [I1, I2, I3] = ind2sub(N, ind); % generate 3d indector https://www.mathworks.com/help/matlab/ref/ind2sub.html
    
    idx_alpha_0  = find(grid.vs{2}==0);
    idx_alpha_pi = find(abs(grid.vs{2}-pi)<eps); % if grid value (2), = pi or 0, when y axis = 0

    idx_theta = find(sin(grid.vs{3}) < -eps); % if robot heading down
    
    I1_reset = I1; % indectors for reset map dim1
    I2_reset = I2; % dim2
    I3_reset = I3; % dim3
    %% Scanning all the grid points and if it's the reset map condition, apply it.
    for j = ind
        i1 = I1(j); % current element indector
        i2 = I2(j);
        i3 = I3(j);
        if (any(i2 == idx_alpha_0) || any(i2 == idx_alpha_pi)) && any(idx_theta == i3)
            % alpha to (alpha + pi)
            if i2 == idx_alpha_0
                i2_post = idx_alpha_pi; % switch the post indector
            elseif i2 == idx_alpha_pi
                i2_post = idx_alpha_0;
            end
            % theta to (theta + pi)
            if i3 <= M
                i3_post = i3 + M; % M=N/2 = pi/2
            else
                i3_post = i3 - M;
            end
            r_bar_current = grid.vs{1}(i1);
            alpha = params.reset_map_nonlinear_factor;
            r_bar_post = R * ((r_bar_current + R)/R)^alpha - R;
            [~, i1_post] = min(abs(grid.vs{1} - r_bar_post)); % swich the x indector based on switched x value

            I1_reset(j) = i1_post;
            I2_reset(j) = i2_post;
            I3_reset(j) = i3_post;
        end        
    end
    reset_map = sub2ind(N, I1_reset, I2_reset, I3_reset); % switch back to 1d indector
end
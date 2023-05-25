clear all
close all

addpath(genpath('D:\matlab_lib\helperOC\'));
addpath(genpath('D:\matlab_lib\ToolboxLS\'));
addpath(genpath('data\'));
addpath(genpath('visualize\'));
addpath(genpath('hj_pde\'));

%% generate target fcn area
R = 2;
M = 40;
params.grid_dx1 = 8 * R / (2 * M);
params.grid_dx3 = 2*pi/(2*M);

params.grid_min = [-4*R + params.grid_dx1; -4*R + params.grid_dx1; -pi];
params.grid_max = [-4*R + params.grid_dx1*2*M; -4*R + params.grid_dx1*2*M; pi];
N = [2*M, 2*M, 2*M]; % M/2 corresponds to pi/2 for 3rd axes
params.index_max = N;
%pdDims = [2 3]; % ?
grid = createGrid(params.grid_min, params.grid_max, N);

% set 3d target fcn
g_tmp_min = params.grid_min(1:2);
g_tmp_max = params.grid_max(1:2);
N_tmp = [N(1); N(2)];
g_tmp = createGrid(g_tmp_min, g_tmp_max, N_tmp);

target_area_r = 0.5;
target_area_pos = [0;4];

data0 = zeros(N);
for i=1:N(3)
    %ToolboxLS\Kernel\InitialConditions\SetOperations, Get combined shape
    data0(:,:,i) = shapeSphere(g_tmp,target_area_pos,target_area_r);
end

%%
params.v = 2; % forward_vel
params.R = R; % cycle_r
params.u_bound = 1; % turning limit
params.alpha = 1.0;

%% Load target set and grid
%load('dubins_target_binary_hybrid')
% Load value function set
%target_function = data0;

%% solver setup
schemeData.grid = grid;
schemeData.uMode = 'min'; % control trying to min the cost fcn, reachable set

schemeData.dynSys = hybird_mod_dubins_car([], [], params, 'parametrized'); % select dyn model

schemeData.accuracy = 'high';
schemeData.hamFunc = @genericHam;
schemeData.partialFunc = @genericPartial;
[schemeData.dissFunc, ~, schemeData.derivFunc] = ...
    getNumericalFuncs('global', schemeData.accuracy);

HJIextraArgs.stopConverge = 1;
HJIextraArgs.targetFunction = data0;

HJIextraArgs.visualize.valueSet = 1;
HJIextraArgs.visualize.initialValueSet = 1;
HJIextraArgs.visualize.figNum = 1; % set figure number
HJIextraArgs.visualize.deleteLastPlot = true; % delete previous plot as you update

%% sim time
t0 = 0;
dt = 0.1;
t_max = 6;
tau = t0:dt:t_max;
%data0 = target_function; % pre-defined target set value function (generated from python script)

alpha = params.alpha; % nonlinear reset para

%% get reset map
schemeData.reset_map = get_reset_map_parametrized(grid, params);
%% cal brt
[data, tau, extraOuts] = ...
        HJIPDE_solve_with_reset_map(data0, tau, schemeData, 'minVWithL', HJIextraArgs);
%% get optimal ctr
%derivatives = computeGradients(grid, data(:,:,:,end));
%safety_controller = schemeData.dynSys.optCtrl([], [], derivatives, 'min'); % min signed dist
%%
data_file_str = strcat('data_with_reset_map_alpha_', num2str(100*alpha));
data_file_str = strcat(data_file_str, '_t_');
data_file_str = strcat(data_file_str, num2str(t_max));
save(strcat(data_file_str, '.mat'), 'grid', 'data0', 'params', 'data', 'tau'); % save data

%% get rest map indector
function reset_map = get_reset_map_parametrized(grid, params)

    eps = 1e-5; % *~0.2, based on grid resolution
    R = params.R;
    N = grid.N; % grid num vector
    M = N(3) / 2;
    ind = 1:prod(N); % how many elements in grid, = N(1)*N(2)*N(3)
    [I1, I2, I3] = ind2sub(N, ind); % generate 3d indector https://www.mathworks.com/help/matlab/ref/ind2sub.html
    
    idx_y_0  = find(abs(grid.vs{2}) < eps) % when car hits y axis
    %idx_alpha_pi = find(abs(grid.vs{2}-pi)<eps); % if grid value (2), = pi or 0, when y axis = 0
    %idx_y_0 = find(abs(grid.vs{2})<eps);

    idx_theta = find(sin(grid.vs{3}) < -eps); % if robot heading down
    
    I1_reset = I1; % indectors for reset map dim1
    I2_reset = I2; % dim2
    I3_reset = I3; % dim3

    post_cnt = 0;
    %% Scanning all the grid points and if it's the reset map condition, apply it.
    for j = ind
        i1 = I1(j); % current element indector
        i2 = I2(j);
        i3 = I3(j);
        if any(i2 == idx_y_0) && any(i3 == idx_theta)
            % keep y
            i2_post = i2;
            % theta to (theta + pi)
            % if i3 <= M % M
            %     i3_post = i3 + M; % M=N/2 = pi/2
            % else
            %     i3_post = i3 - M;
            % end
            % change x
            x_bar_current = grid.vs{1}(i1); % get current x value
            alpha = params.alpha;
            x_bar_post = -1*(abs(x_bar_current)/50) * sign(x_bar_current); % cal new x value
            i1_post = ceil((x_bar_post - params.grid_min(1))/params.grid_dx1) % find new x index
            
            post_cnt = post_cnt+1

            if i1_post < 1
                i1_post = 1;
            elseif i1_post > params.index_max(1)
                i1_post = params.index_max(1);
            end

            theta_bar_current = grid.vs{3}(i3); % get current x value
            if theta_bar_current <= 0
                theta_bar_current = theta_bar_current + pi;
            else
                theta_bar_current = theta_bar_current - pi;
            end
            i3_post = ceil((theta_bar_current - params.grid_min(3))/params.grid_dx3); % find new theta index
            if i3_post < 1
                i3_post = 1;
            elseif i3_post > params.index_max(3)
                i3_post = params.index_max(3);
            end
            I1_reset(j) = i1_post;
            I2_reset(j) = i2_post;
            I3_reset(j) = i3_post;
        end        
    end
    reset_map = sub2ind(N, I1_reset, I2_reset, I3_reset); % switch back to 1d indector
end

function [dissFunc, integratorFunc, derivFunc] = getNumericalFuncs(dissType, accuracy)
% Dissipation
switch(dissType)
    case 'global'
        dissFunc = @artificialDissipationGLF;
    case 'local'
        dissFunc = @artificialDissipationLLF;
    case 'locallocal'
        dissFunc = @artificialDissipationLLLF;
    otherwise
        error('Unknown dissipation function %s', dissType);
end

% Accuracy
switch(accuracy)
    case 'low'
        derivFunc = @upwindFirstFirst;
        integratorFunc = @odeCFL1;
    case 'medium'
        derivFunc = @upwindFirstENO2;
        integratorFunc = @odeCFL2;
    case 'high'
        derivFunc = @upwindFirstENO3;
        integratorFunc = @odeCFL3;
    case 'veryHigh'
        derivFunc = @upwindFirstWENO5;
        integratorFunc = @odeCFL3;
    otherwise
        error('Unknown accuracy level %s', accuracy);
end
end
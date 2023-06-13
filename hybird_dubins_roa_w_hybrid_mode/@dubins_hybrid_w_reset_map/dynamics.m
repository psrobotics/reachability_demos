function dx = dynamics(obj, ~, x, u, ~, q_mode)
% ori State:    x_1 = \bar{r} (r - R), r is the first polar coord. of the car
%           x_2 = alpha, the 2nd polar coord of the car.
%           x_3 = theta, heading of the car

% we're not using polar coord
% dot_x_1 = v cos alpha
% dot_x_2 = v sin alpha
% dot alpha = u

% if there's no operation mode input
if nargin<6
    q_mode = 1;
end

if iscell(x)
    dx = cell(length(obj.dims), 1);    
    for i = 1:length(obj.dims)
        dx{i} = dynamics_cell_helper(obj, x, u, obj.dims(i));
    end
    x
else
    dx = zeros(obj.nx, 1);
    % sometimes the returned u might be cell
    if iscell(u)
        u_num = u{1};
    end
    % pick up control based on q mode
    switch q_mode
        case 1
        dx(1) = 2 * obj.v * cos(x(3)).*is_in_obst(x,obj.obst_range);
        dx(2) = 2 * obj.v * sin(x(3)).*is_in_obst(x,obj.obst_range);
        dx(3) = u_num;
        case 2
        dx(1) = 0.5 * obj.v * cos(x(3)).*is_in_obst(x,obj.obst_range);
        dx(2) = 0.5 * obj.v * sin(x(3)).*is_in_obst(x,obj.obst_range);
        dx(3) = u_num;
    end

end

end

%%
function dx = dynamics_cell_helper(obj, x, u, dim)

% set dynamic based on operation modes
switch q_mode
    case 1
    case 2
end
switch dim
    case 1
        dx = obj.v * cos(x{obj.dims==3}).*is_in_obst(x,obj.obst_range);
    case 2
        dx = obj.v * sin(x{obj.dims==3}).*is_in_obst(x,obj.obst_range);
    case 3
        dx = u;
    otherwise
        error("Dimension index not defined.")
end
end

%% check if the robot is in obstacle
function in_obst_arr = is_in_obst(x, obst_range)

% check if x is cell
if iscell(x)
    x_value_y = x{2}; % x is a 1x1 cell with state matrix inside, copy y value, 80x80x80
end

in_state_size = size(x_value_y);
in_obst_arr = ones(in_state_size);

for idx = 1:numel(in_obst_arr)
    y_n = x_value_y(idx);
    if(y_n > obst_range(1) && y_n <obst_range(2))
        in_obst_arr(idx) = 0; % add velocity limit if in obst range
    end
end

end
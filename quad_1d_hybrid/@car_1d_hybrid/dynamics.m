function dx = dynamics(obj, ~, x, u, d, q_mode)
% Dynamics of the car 1d hybrid
% dynamics 1 
% x_dot = 4*u when x not in obst 
%         0   when x in obst
% dynamics 2
% x_dot = u   for all x
% q_mode is hybrid control mode

%fprintf('params in num %d with q mode %d\n',nargin,q_mode);

% init, if there's no distrubance range input
if nargin < 5
  d = 0;
end

% if there's no operation mode input
if nargin<6
    q_mode = 1;
end

%q_mode = 1

if iscell(x)
  dx = cell(length(obj.dims), 1);
  
  for i = 1:length(obj.dims)
    dx{i} = dynamics_cell_helper(obj, x, u, d, q_mode, obj.dims, obj.dims(i));
  end
% set dyn for non-cell element
else
  dx = zeros(obj.nx, 1);
  switch q_mode
      case 1 % operation mode 1
          dx(1) = 4*u.*is_in_obst(x, obj.obst_range) + d(1);
      case 2 % operation mode 2
          dx(1) = 1*u + d(1);
  end

end
end

% set dyn for cell
function dx = dynamics_cell_helper(obj, x, u, d, q_mode, dims, dim)



switch q_mode
    case 1 % operation mode 1
    dx = 4*u{1}.*is_in_obst(x, obj.obst_range) + d{1}; % mod
    case 2 % operation mode 2
    dx = 1*u{1} + d{1};
  otherwise
    error('Only 2 operation modes available')
end
end


%%
function in_obst_arr = is_in_obst(x, obst_range)
x_value = x{1}; % x is a 1x1 cell with double matrix inside
in_state_size = size(x_value);
in_obst_arr = ones(in_state_size);

for i = 1:in_state_size
    if x_value(i)>obst_range(1) && x_value(i)<obst_range(2)
        in_obst_arr(i) = 0; % add velocity limit if in obst range
    end
end

%in_obst_arr

end
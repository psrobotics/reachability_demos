function dx = dynamics(obj, ~, x, u, d, q_mode)
% Dynamics of the car 1d hybrid
% dynamics 1 
% x_dot = 4*u when x not in obst 
%         0   when x in obst
% dynamics 2
% x_dot = u   for all x
% q_mode is hybrid control mode

% init
if nargin < 5
  d = [0; 0];
end

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
          dx(1) = 4*u(1)*~is_in_obst(x,obj.obst_range) + d(1);
      case 2 % operation mode 2
          dx(1) = 1*u(1) + d(1);
  end

end
end

% set dyn for cell
function dx = dynamics_cell_helper(obj, x, u, d, q_mode, dims, dim)

switch q_mode
    case 1 % operation mode 1
    dx = 4*u(1)*~is_in_obst(x,obj.obst_range) + d{1};
    case 2 % operation mode 2
    dx = 1*u(1) + d{1};
  otherwise
    error('Only 2 operation modes available')
end
end

% check if the robot in the range of obstacle
function [in_obst] = is_in_obst(x, obst_range)
in_obst = 0;
if x > obst_range(1) && x < obst_range(2)
    in_obst = 1;
end
end
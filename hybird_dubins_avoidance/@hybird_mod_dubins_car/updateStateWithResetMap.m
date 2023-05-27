function x_out = updateStateWithResetMap(obj, u, T, x0, d)

% simulate and update the car state, with pre-defined reset map
% x1 = updateState(obj, u, T, x0, d)
% Updates state based on control
%
% Inputs:   obj - current quardotor object
%           u   - control (defaults to previous control)
%           T   - duration to hold control
%           x0  - initial state (defaults to current state if set to [])
%           d   - disturbance (defaults to [])
%
% Outputs:  x1  - final state
%
% Mo Chen, 2015-05-24

% If no state is specified, use current state
if nargin < 4 || isempty(x0)
  x0 = obj.x;
end

% If time horizon is 0, return initial state
if T == 0
  x1 = x0;
  return
end

% Default disturbance
if nargin < 5
  d = [];
end

% Do nothing if control is empty
if isempty(u)
  x1 = x0;
  return;
end

% convert u to vector if needed
if iscell(u)
  u = cell2mat(u);
end

% Do nothing if control is not a number
if isnan(u)
  warning('u = NaN')
  x1 = x0;
  return;
end

% Make sure control input is valid
if ~isnumeric(u)
  error('Control must be numeric!')
end

% Convert control to column vector if needed
if ~iscolumn(u)
  u = u';
end

% Check whether there's disturbance (this is needed since not all vehicle
% classes have dynamics that can handle disturbance)
odeOpts = odeset('Events', @(t,x)dubins_reset_event(t,x));
    
if isempty(d)
  [ts, x] = ode113(@(t,x) obj.dynamics(t, x, u), [0 T], x0, odeOpts);
else
  [~, x] = ode113(@(t,x) obj.dynamics(t, x, u, d), [0 T], x0, odeOpts);
end

% Update the state, state history, control, and control history
obj.u = u;

% get current state, (last one)
x1 = x(end, :)';
[value_end, ~, ~] = dubins_reset_event(ts(end), x1);
%fprintf('vll %d\n',value_end);

if value_end < 0
    fprintf('state_flip\n');
    % Apply reset map
    xPost = zeros(3, 1);
    % R pre-defined traj params
    if strcmp(obj.reset_map_type, 'parametrized')
        %xPost(1) = -1*obj.R * (abs(x1(1))/obj.R)^obj.params.alpha *sign(x1(1));
        xPost(1) = -1 * (abs(x1(1))/4) *sign(x1(1));
    else
        error("reset_map_type not supported.")
    end
    % y
    xPost(2) = x1(2);

    % heading alpha
    if x1(3) <= 0
        xPost(3) = x1(3) + pi;
    else
        xPost(3) = x1(3) - pi;
    end
    obj.x = xPost;
else
    obj.x = x1;
end

% check if heading angle goes beyond range
if obj.x(3) > pi
    obj.x(3) = obj.x(3) - 2*pi;
elseif obj.x(3) < -pi
    obj.x(3) = obj.x(3) + 2*pi;
end

x_out = obj.x;

obj.xhist = cat(2, obj.xhist, obj.x);
obj.uhist = cat(2, obj.uhist, u);
end
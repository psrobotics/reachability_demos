function alpha = genericPartial_hybrid(t, data, derivMin, derivMax, schemeData, dim)
% alpha = genericPartial(t, data, derivMin, derivMax, schemeData, dim)

fprintf("modified genericPartial solver \n");

g = schemeData.grid;
dynSys = schemeData.dynSys;

if ismethod(dynSys, 'partialFunc')
%   disp('Using partial function from dynamical system')
  alpha = dynSys.partialFunc(t, data, derivMin, derivMax, schemeData, dim);
  return
end

if ~isfield(schemeData, 'uMode')
  schemeData.uMode = 'min';
end

if ~isfield(schemeData, 'dMode')
  schemeData.dMode = 'min';
end

% TIdim = [];
% dims = 1:dynSys.nx;
% if isfield(schemeData, 'MIEdims')
%   TIdim = schemeData.TIdim;
%   dims = schemeData.MIEdims;
% end

% x = cell(dynSys.nx, 1);
% x(dims) = g.xs;

%% Compute control
if isfield(schemeData, 'uIn')
  % Control
  uU = schemeData.uIn;
  uL = schemeData.uIn;
 
else
  % Optimal control assuming maximum deriv
  uU = dynSys.optCtrl(t, g.xs, derivMax, schemeData.uMode);
  
  % Optimal control assuming minimum deriv
  uL = dynSys.optCtrl(t, g.xs, derivMin, schemeData.uMode);
end

%% Compute disturbance
if isfield(schemeData, 'dIn')
  dU = schemeData.dIn;
  dL = schemeData.dIn;
  
else
  dU = dynSys.optDstb(t, g.xs, derivMax, schemeData.dMode);
  dL = dynSys.optDstb(t, g.xs, derivMin, schemeData.dMode);
end
  
%% Compute alpha

alpha_arr = [];

for q_mode = 1:dynSys.q_mode_num
    dxUU = dynSys.dynamics(t, schemeData.grid.xs, uU, dU, q_mode);
    dxUL = dynSys.dynamics(t, schemeData.grid.xs, uU, dL, q_mode);
    dxLL = dynSys.dynamics(t, schemeData.grid.xs, uL, dL, q_mode);
    dxLU = dynSys.dynamics(t, schemeData.grid.xs, uL, dU, q_mode);
    alpha_tmp = max(abs(dxUU{dim}), abs(dxUL{dim}));
    alpha_tmp = max(alpha_tmp, abs(dxLL{dim}));
    alpha_tmp = max(alpha_tmp, abs(dxLU{dim}));
    
    alpha_arr = [alpha_arr; alpha_tmp];
    
end

alpha = max(alpha_arr,[],'all'); % pick smallest t-step across all possible cases
% alpha makes sure timestep for each compute won't blow up

end

function dOpt = optDstb(obj, ~, ~, deriv, dMode)
% dOpt = optCtrl(obj, t, y, deriv, dMode)
%     Dynamics of the drone
%         \dot{x}_1 = v * cos(x_3) + d_1
%         \dot{x}_2 = v * sin(x_3) + d_2
%         \dot{x}_3 = u

%% Input processing
if nargin < 5
  dMode = 'max';
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end

dOpt = cell(obj.nd, 1);

%% Optimal control
if strcmp(dMode, 'max')
    dOpt{1} = (deriv{obj.dims==1}>=0) * -1*obj.dRange + (deriv{obj.dims==1}<0) * obj.dRange;
elseif strcmp(dMode, 'min')
    dOpt{1} = (deriv{obj.dims==1}>=0) * obj.dRange + (deriv{obj.dims==1}<0) * -1*obj.dRange;
else
  error('Unknown dMode!')
end

% forcely set to 0
dOpt{1} = 0;

end
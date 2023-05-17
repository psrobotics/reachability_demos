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
    theta_opt = atan2(deriv{2},deriv{1});
    dOpt{1} = obj.dRange*cos(theta_opt); %dx
    dOpt{2} = obj.dRange*sin(theta_opt); %dy
elseif strcmp(dMode, 'min')
    theta_opt = atan2(deriv{2},deriv{1}) + pi;
    dOpt{1} = obj.dRange*cos(theta_opt); %dx
    dOpt{2} = obj.dRange*sin(theta_opt); %dy
else
  error('Unknown dMode!')
end

end
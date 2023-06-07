function uOpt = optCtrl(obj, ~, ~, deriv, uMode)
% uOpt = optCtrl(obj, t, y, deriv, uMode)

%% Input processing
if nargin < 5
  uMode = 'min';
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end

%% TODO
% Compute the optimal control
if strcmp(uMode, 'max')
  uOpt = atan2(deriv{2},deriv{1});% Compute the optimal control for max case
  uOpt(uOpt>obj.thetaMax) = obj.thetaMax;
  uOpt(uOpt<(-1*obj.thetaMax)) = -1*obj.thetaMax;

elseif strcmp(uMode, 'min')
  uOpt = atan2(deriv{2},deriv{1}) + pi;% Compute the optimal control for min case
  uOpt(uOpt>obj.thetaMax) = obj.thetaMax;
  uOpt(uOpt<(-1*obj.thetaMax)) = -1*obj.thetaMax;
  
else
  error('Unknown uMode!')
end

end
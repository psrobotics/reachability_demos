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
if strcmp(uMode, 'max') % when control try to avoid obst
  uOpt = (deriv{obj.dims==3}>=0) * obj.uRange + (deriv{obj.dims==3}<0) * -1*obj.uRange;

elseif strcmp(uMode, 'min') % when control try to reach target set
  uOpt = (deriv{obj.dims==3}>=0) * -1*obj.uRange + (deriv{obj.dims==3}<0) * obj.uRange;
  
else
  error('Unknown uMode!')
end

end
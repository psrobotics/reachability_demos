function uOpt = optCtrl(obj, ~, ~, deriv, uMode)
% uOpt = optCtrl(obj, t, y, deriv, uMode)

%fprintf('the speeed range is %f \n', obj.uRange);

%% Input processing
if nargin < 5
  uMode = 'min';
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end

%% TODO
% Compute the optimal control
uOpt = cell(obj.nu, 1);

if strcmp(uMode, 'max') % when control try to avoid obst
  uOpt{1} = (deriv{obj.dims==1}>=0) * obj.uRange + (deriv{obj.dims==1}<0) * -1*obj.uRange;

elseif strcmp(uMode, 'min') % when control try to reach target set
  uOpt{1} = (deriv{obj.dims==1}>=0) * -1*obj.uRange + (deriv{obj.dims==1}<0) * obj.uRange;
  
else
  error('Unknown uMode!')
end


end
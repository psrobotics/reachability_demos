function hamValue = genericHam(t, data, deriv, schemeData)
% function hamValue = genericHam(t, data, deriv, schemeData)

fprintf("modified hamValue solver \n");

%% Input unpacking
dynSys = schemeData.dynSys;

if ~isfield(schemeData, 'uMode')
  schemeData.uMode = 'min';
end

if ~isfield(schemeData, 'dMode')
  schemeData.dMode = 'max';
end

if ~isfield(schemeData, 'tMode')
  schemeData.tMode = 'backward';
end

% Custom derivative for MIE
if isfield(schemeData, 'deriv')
  deriv = schemeData.deriv;
end

%% Optimal control and disturbance
if isfield(schemeData, 'uIn')
  u = schemeData.uIn;
else
  u = dynSys.optCtrl(t, schemeData.grid.xs, deriv, schemeData.uMode);
end

if isfield(schemeData, 'dIn')
  d = schemeData.dIn;
else
  d = dynSys.optDstb(t, schemeData.grid.xs, deriv, schemeData.dMode);
end

hamValue = 0;
%% MIE
if isfield(schemeData, 'side')
  if strcmp(schemeData.side, 'lower')
%     if schemeData.dissComp
%       hamValue = hamValue - schemeData.dc;
%     end
%     
    TIderiv = -1;
  elseif strcmp(schemeData.side, 'upper')
%     if schemeData.dissComp
%       hamValue = hamValue + schemeData.dc;
%     end
    
%     hamValue = -hamValue;
%     deriv{1} = -deriv{1};
% %     if schemeData.trueMIEDeriv
% %       deriv{2} = -deriv{2};
% %     end
    TIderiv = 1;
  else
    error('Side of an MIE function must be upper or lower!')
  end
  
%   if ~schemeData.trueMIEDeriv
%     deriv(2) = computeGradients(schemeData.grid, data);
%   end
end

%% Plug optimal control into dynamics to compute Hamiltonian

% changed to select opti q mode, and store it
% calcuate hamvalue for different q mode

hamValue_1 = 0;
hamValue_2 = 0;

q_mode = 1;
dx_1 = dynSys.dynamics(t, schemeData.grid.xs, u, d, q_mode);
for i = 1:dynSys.nx
  fprintf('ham value 1\n');
  hamValue_1 = hamValue_1 + deriv{i}.*dx_1{i};
end

q_mode = 2;
dx_2 = dynSys.dynamics(t, schemeData.grid.xs, u, d, q_mode);
for i = 1:dynSys.nx
  fprintf('ham value 2\n');
  hamValue_2 = hamValue_2 + deriv{i}.*dx_2{i};
end

% select the smaller one, min over each row and return the smallest col
[hamValue, q_opti] = min([hamValue_1,hamValue_2],[],2);
% store thr q opti
schemeData.q_mode_arr(end+1) = q_opti;

% other user selected case, ?
if isprop(dynSys, 'TIdim') && ~isempty(dynSys.TIdim)
  TIdx = dynSys.TIdyn(t, schemeData.grid.xs, u, d);
  hamValue = hamValue + TIderiv*TIdx{1};
end


%% Negate hamValue if backward reachable set
if strcmp(schemeData.tMode, 'backward')
  hamValue = -hamValue;
end

if isfield(schemeData, 'side')
  if strcmp(schemeData.side, 'upper')
    hamValue = -hamValue;
  end
end
end
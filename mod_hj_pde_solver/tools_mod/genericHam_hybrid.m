function hamValue = genericHam_hybrid(t, data, deriv, schemeData)
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

hamValue_arr = ones([schemeData.grid.N,2]); % each col is a q mode, we have 2 modes in this demo

q_mode_count = 2; % we have 2 operation modes

for q_mode = 1:q_mode_count

    dx_tmp = dynSys.dynamics(t, schemeData.grid.xs, u, d, q_mode);
    hamValue_tmp = 0;
    for i = 1:dynSys.nx
        fprintf('ham value %d\n', q_mode);
        hamValue_tmp = hamValue_tmp + deriv{i}.*dx_tmp{i};
    end
    hamValue_arr(:,q_mode) = hamValue_tmp;
end

% select the smaller one, min over row and return the smallest col
[hamValue, q_opti] = min(hamValue_arr, [], 2); 

size(q_opti)
size(hamValue)

% store thr q opti, need to return this
%schemeData.q_mode_arr(:,schemeData.q_index) = q_opti;
%schemeData.q_index = schemeData.q_index+1;

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
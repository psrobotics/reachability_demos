classdef car_1d_hybrid < DynSys
  properties
    % input abs range
    uRange
    % Disturbance abs range
    dRange
    % Dimensions that are active
    dims
    % range of obstacle
    obst_range % 1x2 vec
    % number of all operation modes
    q_mode_num
  end
  
  methods
      function obj = car_1d_hybrid(x, uRange, dRange, q_mode_num, obst_range, dims)
      
      % Basic properties
      % obj.pdim = [find(dims == 1)]; % Position dimensions
      obj.nx = length(dims); % state dim
      obj.nu = 1; % input dim
      obj.nd = 1; % disturb dim
      
      obj.x = x;
      obj.xhist = obj.x;

      obj.uRange = uRange;
      obj.dRange = dRange;
      obj.dims = dims;
      obj.obst_range = obst_range;
      obj.q_mode_num = q_mode_num;
    end
    
  end % end methods
end % end classdef

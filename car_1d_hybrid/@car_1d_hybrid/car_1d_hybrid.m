classdef car_1d_hybrid < DynSys
  properties
    % Disturbance
    dRange
    % Dimensions that are active
    dims
    % range of obstacle
    obst_range % 1x2 vec
  end
  
  methods
      function obj = car_1d_hybrid(x, dRange, dims)
      
      % Basic properties
      obj.pdim = [find(dims == 1)]; % Position dimensions
      obj.nx = length(dims); % state dim
      obj.nu = 1; % input dim
      obj.nd = 1; % disturb dim
      
      obj.x = x;
      obj.xhist = obj.x;

      obj.dRange = dRange;
      obj.dims = dims;
      obj.obst_range = obst_range;
    end
    
  end % end methods
end % end classdef

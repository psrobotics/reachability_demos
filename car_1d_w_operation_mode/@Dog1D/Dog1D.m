classdef Dog1D < DynSys
  properties
    % Control bounds
    uMax           
    nq = 2; %number of possible transitions
  end % end properties
 
  methods
    function obj = Dog1D(x, uMax)
    
      % Process control range
      if nargin < 2
        uMax = 1;
      end
      
      obj.uMax = uMax;
          
      if numel(x) ~= 1
        error('Initial state does not have right dimension!');
      end      
      
      % Process initial state
      obj.x = x;
      obj.xhist = x;
      
      obj.nx = 1;
      obj.nu = 1;
      obj.nd = 0; 
      
      
    end % end constructor    
    
  end % end methods
end % end class
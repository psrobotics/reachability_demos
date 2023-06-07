classdef drone_2d < DynSys
  properties
    % Angle bounds
    thetaMax
    % Constant speed
    speed 
    % Disturbance
    dRange
    % Dimensions that are active
    dims
  end
  
  methods
      function obj = drone_2d(x, thetaMax, speed, dRange, dims)
      % obj = drone_2d(x, thetaMax, speed, dMax, dims)
      %     drone 2d class
      %
      % Dynamics:
      %    \dot{x}_1 = v * cos(theta) + d1
      %    \dot{x}_2 = v * sin(theta) + d2
      %    theta \in [-thetaMax, thetaMax]
      %    d \in [-dMax, dMax]
      %
      % Inputs:
      %   x      - state: [xpos; ypos]
      %   thetaMin   - minimum angle
      %   thetaMax   - maximum angle
      %   v - speed
      %   dMax   - disturbance bounds
      %
      % Output:
      %   obj       - a drone 2d object
      
      % Basic properties
      obj.pdim = [find(dims == 1) find(dims == 2)]; % Position dimensions
      obj.nx = length(dims);
      obj.nu = 1;
      obj.nd = 2;
      
      obj.x = x;
      obj.xhist = obj.x;

      obj.thetaMax = thetaMax;
      obj.speed = speed;
      obj.dRange = dRange;
      obj.dims = dims;
    end
    
  end % end methods
end % end classdef

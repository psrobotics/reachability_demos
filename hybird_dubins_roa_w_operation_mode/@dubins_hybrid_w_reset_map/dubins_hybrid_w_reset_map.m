classdef dubins_hybrid_w_reset_map < DynSys
    properties
        uMax
        uMin        
        v
        R % nonlinear params
        dims
        params
        reset_map_type
    
        % range of obstacle for y axis
        obst_range % 1x2 vec
        % number of all operation modes   
        q_mode_num

    end
    methods
        function obj = dubins_hybrid_w_reset_map(dims, x, params, reset_map_type, q_mode_num, obst_range)
            if nargin < 1 || isempty(dims)
                dims = 1:3;
            end
            if nargin < 2 || isempty(x)
                disp("setting default sate");
                x = [0; 0; pi/2];
            end
            if nargin < 3
                % default parameters
                params.v = 1;
                params.R = 1;
                params.u_bound = 1;
            end
            if nargin < 4
                reset_map_type = "identity";
            end
            
            obj.params = params;
            obj.uMax = params.u_bound;
            obj.uMin = -params.u_bound;
            obj.v = params.v;
            obj.R = params.R;
            obj.reset_map_type = reset_map_type;
            
            obj.nu = 1;            
            obj.x = x;
            obj.dims  = dims;
            obj.nx = length(dims);

            % copy the operation mode and obstcale ranges
            obj.obst_range = obst_range;
            obj.q_mode_num = q_mode_num;
        end
    end
end

                
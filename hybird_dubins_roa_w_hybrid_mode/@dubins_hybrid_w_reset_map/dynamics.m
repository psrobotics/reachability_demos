function dx = dynamics(obj, ts, x, u, d, q)
% ori State:    x_1 = \bar{r} (r - R), r is the first polar coord. of the car
%           x_2 = alpha, the 2nd polar coord of the car.
%           x_3 = theta, heading of the car

% we're not using polar coord
% dot_x_1 = v cos alpha
% dot_x_2 = v sin alpha
% dot alpha = u

% if there's no operation mode input
if nargin<6
    q = 1;
end

%% For reachable set computations
if iscell(x)
    dx = cell(3,1);
    switch q
        case 1
            frz=(x{2}>=2)|(x{2}<=1); %dynamics freeze condition 1<y<2
            dx{1} = 2.0*cos(x{3}).*frz;
            dx{2} = 2.0*sin(x{3}).*frz;
            dx{3} = u.*frz; 
        case 2
            dx{1} = 0.5*cos(x{3});
            dx{2} = 0.5*sin(x{3});
            dx{3} = u; 
        otherwise
            error('q-state not defined for system dynamics')
    end
end


%% For simulations
if isnumeric(x)
  dx = zeros(3,1);
    switch q
        case 1
            frz=(x(2)>=2)|(x(2)<=1); %dynamics freeze condition 1<y<2
            dx(1) = 2.0*cos(x(3)).*frz;
            dx(2) = 2.0*sin(x(3)).*frz;
            dx(3) = u.*frz; 
        case 2
            dx(1) = 0.5*cos(x(3));
            dx(2) = 0.5*sin(x(3));
            dx(3) = u; 
        otherwise
            error('q-state not defined for system dynamics')
    end
end

end
function dx = dynamics(obj, t, x, u, d, q)
% function dx = dynamics(t, x, u)
%     Dynamics of the Dog1D system

if nargin < 6  %if no q especified (GenericHam compatible)
    q = 1;  %default to dynamics 1
end

%% For reachable set computations
if iscell(x)
    dx = cell(1,1);
    switch q
        case 1
            %dx{1} = 3*u;
            dx{1} = 4*u.*((x{1}<-1)|(x{1}>0));
            %dx{1} = 1*u;
        case 2
            dx{1} = 1*u.*(x{1}>-100);
            %dx{1} = 3*u;
        otherwise
            error('q-state not defined for system dynamics')
    end
end


%% For simulations
if isnumeric(x)
  %dx = zeros(1,1);
  error('not implemented!')
  %dx(1) = 4*u;
end

end

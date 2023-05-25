function [value, isterminal, direction] = dubins_reset_event(t, x)

% check if the current state triggers the reset event
% if y<0 && heading down

if x(2) < 0 && sin(x(3)) < 0
    value = -1;
else
    value = 1;
end

isterminal = 1;
direction = 1;
end
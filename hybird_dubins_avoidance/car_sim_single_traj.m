function [x_arr, ctr_arr, t_arr] = car_sim_single_traj(car_sys, x_init, grid, data, sim_time, params)
% sim opti traj of the car
x_arr = x_init;
% for this demo, user pre-set the sim time
t_arr = 1:params.dt:sim_time;
ctr_arr = [];
% get optimal ctr once, since we're not doing roa
derivatives = computeGradients(grid, data(:,:,:,end));
safety_controller = car_sys.optCtrl([], [], derivatives, 'max'); % control tries to max signed dist

for t_n = t_arr
    
    x_n = x_arr(:,end);
    % cal pid controllrt
    r = sqrt(x_n(1)^2+x_n(2)^2);
    alpha = atan2(x_n(2),x_n(1));
    theta = x_n(3);

    dynSys.v = params.v;
    dynSys.R = params.R;

    x_m = [r;alpha;theta];

         if x_m(2) == x_m(3)
             pid_ctr = 1;
         else
             eps = 5;
             k_r = 20;
             k_rdot = 24;             
             mu = -k_r * x_m(1) / eps^2 - k_rdot * dynSys.v * cos(x_m(2)-x_m(3)) / eps;
             u_ff = - sin(x_m(2)-x_m(3)) / (x_m(1) + dynSys.R);
             u_fb = mu / (dynSys.v * sin(x_m(2)-x_m(3)));
             pid_ctr = u_ff + u_fb;
         end
         pid_ctr = min(pid_ctr, 1);
         pid_ctr = max(pid_ctr, -1);
         disp(pid_ctr)

    % sim next step with pid controller first
    x_next = car_sys.updateStateWithResetMap(pid_ctr, params.dt, x_n);
    % cal next signed distance
    next_signed_dist = eval_u(grid, data(:,:,:,end), x_next)

    next_ctr = 0;

    if next_signed_dist < 0.2 % hit brt, apply safety controller
        next_ctr = eval_u(grid,safety_controller,x_n);
    else
        next_ctr = pid_ctr;
    end

    % sim next step with ctr
    x_next = car_sys.updateStateWithResetMap(next_ctr, params.dt, x_n);
    x_arr = [x_arr, x_next];
    ctr_arr = [ctr_arr, next_ctr];

    fprintf('simulating timestep %f\n', t_n);
end

end
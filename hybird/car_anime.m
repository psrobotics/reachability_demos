function [] = car_anime(x_arr, ctr_arr, t_arr, brs_t_ind_arr, g, data, params)
% get brt overlay
BRT_2d_layer = zeros(20,10);
ind = 1:20*10;
[I1, I2] = ind2sub([20,10], ind);

for j = 1:ind(end)
    i1 = I1(j); % current element indector
    i2 = I2(j);
    BRT_2d_layer(i1,i2) = eval_u(g, data(:,:,:,end),...
        [i1*params.grid_dx1*4+g.min(1), i2*params.grid_dx1*4, x_arr(3,1)]);
    fprintf('analysis 2D BRT index %d of %d \n',j,ind(end));
end

% create simulation amine
size_arr = size(x_arr);
len = size_arr(2)-1;

%%
for k = 1:len
    clf;
    hold on;
    axis equal;
    grid on;

    x_t = x_arr(:,k);
    r_mat = rot_zyx([0,0,x_t(3)]);

    view(30,40);
    %view(90,0)
    axis([-8,8,0,8,0,1.5]);

    % plot current car pos
    plot_cube(r_mat,1.1,0.6,0.2,[x_t(1);x_t(2);0.8],'black',2);

    % plot past traj
    for i = 1:k
        x_i = x_arr(:,i);
        r_mat_old = rot_zyx([0,0,x_i(3)]);
        plot_cube(r_mat_old,0.2,0.2,0.2,[x_i(1);x_i(2);0.8],'blue',1);
    end

    [X,Y] = meshgrid(g.min(1): params.grid_dx1*4: g.max(1),...
                      0+params.grid_dx1: params.grid_dx1*4: g.max(2));
    [Xq,Yq] = meshgrid(g.min(1): params.grid_dx1: g.max(1),...
                      0+params.grid_dx1: params.grid_dx1: g.max(2));
    BRT_2d_layer_high_res = griddata(X,Y,BRT_2d_layer',Xq,Yq,"cubic");
    surf(Xq,Yq,-1*BRT_2d_layer_high_res)
    colormap summer

    pause(params.dt);
    hold on;

end

end
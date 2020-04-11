function animate
    
    % Interpolate trajectroy for animation
    load trajectory
    t2 = min(t):0.02:max(t);
    x = interp1(t,x,t2);
    
    
    % Figure setup
    figure(1)
    clf
    hold on
    grid on
    box on
    fig = gcf;
    ax = gca;
    fig.Position = [150 50 1300 670];
    fig.Color = [1 1 1];
    ax.Position = [0.01 0.01 0.98 0.98];
    ax.Toolbar.Visible = 'off';
    axis equal
    xlim([-4 10]+2)
    ylim([-5 2]-1)
    ax.XTickLabel = [];
    ax.YTickLabel = [];
    
    [~, ~, idx] = drone_ode_info;
    
    % Draw trajectory
    plot(x(:,idx.position_x), x(:,idx.position_y), 'r');
    
    % Draw obstacle
    a = linspace(0,2*pi);
    plot(1.7*cos(a)+5, 1.7*sin(a)-3, 'k');

    drone_surf = surf(...
        4/8*[-1 1; -1 1],...
        1/8*[1 1; -1 -1],...
        0*[1 1;1 1],...
        imread('multirotor_lowres.jpg'), ...
        'FaceColor','texturemap',...
        'EdgeColor','none' ...
    );

    thrust_arrows_plot = plot(0,0,'r','LineWidth',2);

    drone_transform = hgtransform('Parent', ax);
    drone_surf.Parent = drone_transform;
    thrust_arrows_plot.Parent = drone_transform;
    
    
    for i = 1:size(x,1)
        x_now = x(i, :);
        
        px = x_now(idx.position_x);
        py = x_now(idx.position_y);
        pitch = x_now(idx.pitch);
        c = cos(pitch);
        s = sin(pitch);
        Tl = x_now(idx.thrust_left);
        Tr = x_now(idx.thrust_right);
        
        drone_transform.Matrix = ...
        [[ c,  -s,   0,  px]; ...
         [ s,   c,   0,  py]; ...
         [ 0,   0,   1,   0]; ...
         [ 0,   0,   0,   1]];

        thrust_arrows_plot.XData = [-1 -1 nan 1 1]*0.27;
        thrust_arrows_plot.YData = [0 Tl/10 nan 0 Tr/10]+0.2;
        
        pause(0.02)
        drawnow
    end
    
end


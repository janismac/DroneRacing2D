function dxdt = drone_ode(x, u)
    
    [~, ~, idx] = drone_ode_info;
    
    %% Unpack the state and input vectors
    velocity        = x(:, idx.velocity);
    pitch           = x(:, idx.pitch);
    pitch_rate      = x(:, idx.pitch_rate);
    thrust_left     = x(:, idx.thrust_left);
    thrust_right    = x(:, idx.thrust_right);
    
    thrust_rate_left  = u(:, idx.thrust_rate_left);
    thrust_rate_right = u(:, idx.thrust_rate_right);
    
    %% Parameters
    mass = 1;
    moment_of_inertia = 0.02;
    moment_arm = 0.2;
    gravitational_acceleration = 9.81;

    %% Equations of motion
    body_direction_up = [-sin(pitch), cos(pitch)];
    direction_up = [zeros(size(x,1),1) ones(size(x,1),1)];
    
    acceleration = ...
        body_direction_up .* (thrust_left + thrust_right) ./ mass ...
        + direction_up .* gravitational_acceleration;
    
    pitch_acceleration = (thrust_right - thrust_left) .* moment_arm ./ moment_of_inertia;
    
    %% Pack the derivative vector
    dxdt = 0*x;
    dxdt(:, idx.position)       = velocity;
    dxdt(:, idx.velocity)       = acceleration;
    dxdt(:, idx.pitch)          = pitch_rate;
    dxdt(:, idx.pitch_rate)     = pitch_acceleration;
    dxdt(:, idx.thrust_left)    = thrust_rate_left;
    dxdt(:, idx.thrust_right)   = thrust_rate_right;
    
end


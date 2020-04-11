function simulate
    
    [n_states, n_inputs, idx] = drone_ode_info;
    initial_state = zeros(n_states, 1);
    initial_state(idx.position_x) = -3;
    initial_state(idx.thrust_left) = 5;
    initial_state(idx.thrust_right) = 5;
    
    [t,x] = ode45(@(t,y) closed_loop_ode(y), 0:0.01:10, initial_state);
    save trajectory x
    animate
end

function dxdt = closed_loop_ode(x)
    u = drone_controller(x');
    dxdt = drone_ode(x', u)';
end

function u = drone_controller(x)
    
    [~, ~, idx] = drone_ode_info;
    
    %% Unpack the state and input vectors
    position        = x(:, idx.position);
    velocity        = x(:, idx.velocity);
    pitch           = x(:, idx.pitch);
    pitch_rate      = x(:, idx.pitch_rate);
    thrust_left     = x(:, idx.thrust_left);
    thrust_right    = x(:, idx.thrust_right);
    
    
    %% Cascade control
    target_position = [6 3];
    position_error = position - target_position;
    target_velocity = -0.5 * position_error;
    velocity_error = velocity - target_velocity;
    target_acceleration = [0 9.81] -1.5 * velocity_error;
    target_pitch = atan2(-target_acceleration(1), target_acceleration(2));
    target_acceleration_magnitude = norm(target_acceleration);
    pitch_error = pitch - target_pitch;
    target_pitch_rate = -3 * pitch_error;
    pitch_rate_error = target_pitch_rate - pitch_rate;
    
    target_thrust_left =  -1 * pitch_rate_error + target_acceleration_magnitude/2;
    target_thrust_right =  1 * pitch_rate_error + target_acceleration_magnitude/2;
    
    u(1, idx.thrust_rate_left) = 20 * (target_thrust_left - thrust_left);
    u(1, idx.thrust_rate_right) = 20 * (target_thrust_right - thrust_right);
    
    
end
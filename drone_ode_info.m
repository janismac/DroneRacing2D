function [n_states, n_inputs, idx] = drone_ode_info
    
    idx = struct;
    
    %% Indices of the state vector entries
    idx.position = [1 2];
    idx.position_x = 1;
    idx.position_y = 2;
    
    idx.velocity = [3 4];
    idx.velocity_x = 3;
    idx.velocity_y = 4;
    
    idx.pitch = 5;
    idx.pitch_rate = 6;
    
    idx.thrust_left = 7;
    idx.thrust_right = 8;

    %% Indices of the input vector entries
    idx.thrust_rate_left = 1;
    idx.thrust_rate_right = 2;
    
    %% Vector sizes
    n_states = 8;
    n_inputs = 2;

end


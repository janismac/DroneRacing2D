function optimize

    clc
    opti = casadi.Opti();
    [n_states, n_inputs, idx] = drone_ode_info;
    n_timesteps = 300;

    %% Setup optimization variables
    delta_t = opti.variable();
    state_trajectory = opti.variable(n_timesteps, n_states);
    input_trajectory = opti.variable(n_timesteps, n_inputs);
    
    %% Evaluate the model ODE
    dxdt = drone_ode(state_trajectory, input_trajectory);
    
    %% Use the trapezoidal rule for the discretization
    % https://en.wikipedia.org/wiki/Trapezoidal_rule_(differential_equations)
    delta_x = state_trajectory(2:end,:) - state_trajectory(1:end-1,:);
    opti.subject_to(delta_x == 0.5 * delta_t * (dxdt(2:end,:) + dxdt(1:end-1,:)));
    
    
    %% Optimization constraints and objectives
    
    % Initial state
    opti.subject_to(state_trajectory(1, idx.position_x) == 0);
    opti.subject_to(state_trajectory(1, idx.position_y) == 0);
    opti.subject_to(state_trajectory(1, idx.velocity_x) == 0);
    opti.subject_to(state_trajectory(1, idx.velocity_y) == 0);
    opti.subject_to(state_trajectory(1, idx.pitch) == 0);
    opti.subject_to(state_trajectory(1, idx.pitch_rate) == 0);
    opti.subject_to(dxdt(1, idx.velocity_x) == 0);
    opti.subject_to(dxdt(1, idx.velocity_y) == 0);
    opti.subject_to(dxdt(1, idx.pitch_rate) == 0);
    
    % Final state
    opti.subject_to(state_trajectory(end, idx.position_x) == 6);
    opti.subject_to(state_trajectory(end, idx.position_y) == 3);
    opti.subject_to(state_trajectory(end, idx.velocity_x) == 0);
    opti.subject_to(state_trajectory(end, idx.velocity_y) == 0);
    opti.subject_to(state_trajectory(end, idx.pitch) == 0);
    opti.subject_to(state_trajectory(end, idx.pitch_rate) == 0);
    opti.subject_to(dxdt(end, idx.velocity_x) == 0);
    opti.subject_to(dxdt(end, idx.velocity_y) == 0);
    opti.subject_to(dxdt(end, idx.pitch_rate) == 0);
    
    % Time must run forwards
    opti.subject_to(0.01 < delta_t < 1);
    
    % Objectives
    opti.minimize(...
           100 * delta_t ...                    % Minimize time to target
        + 1e-4 * sum(input_trajectory(:).^2));  % Minimize thrust variation
    

    
    
    %% Run the optimization
    % Set some initial guesses first
    opti.set_initial(delta_t, 0.1);
    opti.set_initial(state_trajectory, 0);
    opti.set_initial(input_trajectory, 0);
    opti.set_initial(state_trajectory(:, idx.thrust_left), 9.81/2);
    opti.set_initial(state_trajectory(:, idx.thrust_right), 9.81/2);
    opti.solver('ipopt');
    sol = opti.solve();
    
    
    %% Save and animate the result
    x = sol.value(state_trajectory);
    save trajectory x
    animate
end


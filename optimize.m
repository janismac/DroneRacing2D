function optimize

    opti = casadi.Opti();
    [n_states, n_inputs, ~] = drone_ode_info;
    n_timesteps = 40;

    %% Setup optimization variables
    delta_t = opti.variable();
    state_trajectory = opti.variable(n_timesteps, n_states);
    input_trajectory = opti.variable(n_timesteps, n_inputs);
    
    %% Evaluate the model ODE
    dxdt = drone_ode(state_trajectory, input_trajectory);
    
    %% Use the trapezoidal rule for the discretization
    % https://en.wikipedia.org/wiki/Trapezoidal_rule_(differential_equations)
    delta_x = state_trajectory(2:end) - state_trajectory(1:end-1);
    opti.subject_to(delta_x == 0.5 * delta_t * (dxdt(2:end) + dxdt(1:end-1)));
    
    %% Objective: Minimize the time to the target
    opti.minimize(delta_t);
    
    %% Limit thrust rate for smooth flight
    

    
end


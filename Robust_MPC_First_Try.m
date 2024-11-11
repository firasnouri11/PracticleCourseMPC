%x(t + 1) = Ax(t) + Bu(t) + w(t)

% Defining the matrixes of the sytem for the system: x(t + 1) = Ax(t) + Bu(t) + w(t)

A = [1 1; 0 1];           %state transition matrix ; 
% how x(t) evolves over time 

B = [0.5; 1];            %Control inpuit matrix
%how the control input u(t) affects the state x(t)
 
x0 = [2; 0];             % Initial state ; first state value is 2 and second state value is 0
% starting point of the system, from which the simulation will begin


% Define Cost Function Parameters
Q = eye(2);              % State cost matrix  ; eye generates 2x2 IDENTITY matrix ; both state variables are penalized equally in the cost function. (1 0 / 0 1)
R = 1;                   % Control cost matrix (scalar for single input);  moderate penalty

%Q : penalizes deviations in the state
%R: penalizes the control effort

% Define Constraints
u_max = 1;               % Maximum allowable control input ; u cannot exceed 1
x_max = [10; 10];        % Maximum allowable state values ; maximum allowable values for each state variable


% Define the Uncertainty Set using Zonotopes
disturbance = 0.1 * [-1 1];  % zonotopes 0.1 or -0.1 , will be chosen randomly later 
%At each time step, the disturbance can randomly take a value of either -0.1 or 0.1, introducing small random variations to the state.



% Set up Time Horizon
N = 5;                   % Short prediction horizon to keep things simple
T = 15;                  % Total simulation steps

% Initialize Variables
x = x0;                  % Current state
x_traj = x0;             % State trajectory for plotting
u_traj = [];             % Control trajectory for plotting

% MPC Loop
for t = 1:T

    u = -0.5 * x(1);     % K=0.5 the proortional gain and x1 is the first state variable   
    
% achieve stability:       simple control strategy to stabilize the system.


    % Apply constraints to the control input
  u = max(min(u, u_max), -u_max);  % Limit control input to allowable range
    

% ensures that the control input u stays within the allowable range defined  umax and -umax


    % Compute Next State (using system dynamics)
    w = disturbance(randi(2));      % Randomly pick a disturbance from the set , either 0.5 or -0.5
    x_next = A * x + B * u + w;     % Update the state with control input and disturbance 
    
    % Apply state constraints ; min max approach used in Robust MPC
   x_next = max(min(x_next, x_max), -x_max);  % Limit the state to within allowable bounds
    

% min(x_next, x_max) compares each element of x_next with x_max. If any element of x_next exceeds the corresponding element in x_max, it is set to the value in x_max, limiting it from above.
% max(..., -x_max) then takes the result and ensures that no element of x_next is below -x_max, limiting it from below.
% All this in order  to make sure that  each element of x_next to be within the range [ -x_max, x_max ].



    % Store results for plotting
    x_traj = [x_traj, x_next];       % Append new state to trajectory
    u_traj = [u_traj, u];            % Append control input to trajectory
    
    % Update current state for the next iteration
    x = x_next;
end



% Plot Results
figure;
subplot(2, 1, 1);
plot(0:T, x_traj(1, :), '-o');
title('State x1 over Time');
xlabel('Time Step');
ylabel('x1');

subplot(2, 1, 2);
plot(0:T, x_traj(2, :), '-o');
title('State x2 over Time');
xlabel('Time Step');
ylabel('x2');

% Plot Control Input
figure;
plot(0:T-1, u_traj, '-o');
title('Control Input over Time');
xlabel('Time Step');
ylabel('Control Input u');

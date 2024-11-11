% Import the CORA toolbox
addpath(genpath('C:\Users\firas\CORA'));

% Define the center and generators of the initial zonotope
center = [1; 0]; % Center of the zonotope
generators = [0.5, 0; 0, 0.5]; % Generators for the zonotope

% Create the initial zonotope
Z0 = zonotope(center, generators);

% Plot the initial zonotope
figure;
plot(Z0, [1, 2], 'r'); % Plot using dimensions 1 and 2
title('Initial Zonotope');
xlabel('x_1');
ylabel('x_2');
grid on;

% Define a simple linear system: x' = Ax
A = [0 1; -1 0]; % System matrix
B = [0; 1];      % Input matrix

% Create a linearSys object without inputs for now
sys = linearSys('SimpleSystem', A, [], [], [], B);

% Define input set U as a zonotope
U = zonotope(0, 0.1); % Center 0, generator 0.1

% Set options for reachability analysis
options.timeStep = 0.1; % Time step size
options.tFinal = 1;     % Final time for simulation
options.taylorTerms = 4; % Taylor terms for precision
options.zonotopeOrder = 10; % Zonotope order
options.intermediateOrder = 5; % Order for intermediate computations
options.errorOrder = 2; % Error zonotope order

% Perform reachability analysis, including input
reachSet = reach(sys, Z0, U, options);

% Plot the reachable sets
figure;
hold on;
for i = 1:length(reachSet)
    plot(reachSet{i}, [1, 2], 'b'); % Plot in 2D for states x1 and x2
end
title('Reachable Sets Using Zonotopes');
xlabel('x_1');
ylabel('x_2');
grid on;
hold off;
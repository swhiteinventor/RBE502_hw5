%% TwoLinkArm
% This script simulates a non-planar two-link arm tracking a cubic
% polynomial trajectory with three different tracking controllers: inverse
% dynamic control, Lyapunov-based Control, and passivity-based control.
% When an initial error was introduced, it can be concluded that while all three
% controllers do converge to the desired trajectory, both the
% Lyapunov-based control and passivity-based control converged much faster
% than the inverse dynamic control.
% 
% Clean up before running anything:
clc, clear all, close all
%%
% Initial state:
x0 = [-0.5,0.2,0.1,0.1];
%%
% final state:
xf = [0, 0, 0, 0];
%% 
% Final time:
tf = 10;
%% 
% Option for no figure for the trajectory generator (true/false):
nofigure = 1;
%% 
% Compute the cubic polynomial coefficients:
a1 = TwoLinkArmTraj(x0(1), x0(3), xf(1), xf(3), tf, nofigure);
a2 = TwoLinkArmTraj(x0(2), x0(4), xf(2), xf(4), tf, nofigure);

%%
% Set the initial error state:
x0_error  = [-0.6,0.4,0.15,0.05];
%% 
% Options for plotting each controller (true/false):
plot_inverseDC = true;
plot_lyapunov = true;
plot_passivity = true;

%% Trajectory Generation For Plotting:
% Set time matrix for plotting:
time = 0:0.001:tf;

% Initialize the trajectory maxtrix:
trajectory = zeros(length(time),2);
length(time);
for i = 1:length(time)
    %% 
    % Grab the time value to use for this iteration:
    t = time(1,i);
    %% 
    % Note _x_ is in the form of _q1_, _q2_, _q1_dot_, _q2_dot_:
    % 
    % Cubic polynomials:
    vec_t = [1; t; t^2; t^3];
    theta_d = [a1'*vec_t; a2'*vec_t];
    %% 
    % Save trajectory joint angles for each time:
    trajectory(i,:) = theta_d';
end

if plot_inverseDC
    %% Inverse Dynamic Control
    % Set the tolerance options for ODE45 function:
    options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
    %%
    % Simulate the tracking controller with no initial error:
    [T,X] = ode45(@(t,x) inverseDC(t, x, a1, a2), [0 tf], x0, options);
    [T_error,X_error] = ode45(@(t,x) inverseDC(t, x, a1, a2), [0 tf], x0_error, options);
    
    % Plot the results of the simulations:
    plotTrajectories(1, 'Inverse Dynamic', time, trajectory(:,1), T, X(:,1), T_error, X_error(:,1));
    plotTrajectories(2, 'Inverse Dynamic', time, trajectory(:,2), T, X(:,2), T_error, X_error(:,2));

end
if plot_lyapunov
    %% Implement the lyapunov-based control
    options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
    [T,X] = ode45(@(t,x) lyapunovCtrl(t, x, a1, a2),[0 tf],x0, options);
    
    %plotting
    plotTrajectories(1, 'lyapunov-based', time, trajectory(:,1), T, X(:,1));
    plotTrajectories(2, 'lyapunov-based', time, trajectory(:,2), T, X(:,2));
    
end

if plot_passivity
    %% Implement the passivity-based control
    %initialize A matrix (joint accelerations) as a global variable
    global A
    A = [0;0];
    options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
    [T,X] = ode45(@(t,x) passivityCtrl(t,x, a1, a2),[0 tf],x0, options);
    
    %plotting
    plotTrajectories(1, 'passivity-based', time, trajectory(:,1), T, X(:,1));
    plotTrajectories(2, 'passivity-based', time, trajectory(:,2), T, X(:,2));
    
end
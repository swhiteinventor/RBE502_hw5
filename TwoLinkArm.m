% Notations: For a given variable, x, dx is its time derivative, ddx is
% 2nd-order derivative.
clc, clear all, close all

% initial condition % feel free to change
x0 = [-0.5,0.2,0.1,0.1];
xf = [0, 0, 0, 0];

%final time
tf = 10;

%no figure?
nofigure = 1;

%compute the cubic polynomial coefficients
a1 = TwoLinkArmTraj(x0(1), x0(3), xf(1), xf(3), tf, nofigure);
a2 = TwoLinkArmTraj(x0(2), x0(4), xf(2), xf(4), tf, nofigure);

x0  = [-0.6,0.4,0.15,0.05];

plot_inverseDC = false;
plot_lyapunov = false;
plot_passivity = true;

%% trajectory generation for plotting

%set time matrix for plotting
time = 0:0.001:tf;

%initialize the trajectory maxtrix
trajectory = zeros(length(time),2);
length(time);
for i = 1:length(time)
    %grabs time value to use for this iteration
    t = time(1,i);
    % note x is in the form of q_1, q_2,dot q_1, dot q_2
    vec_t = [1; t; t^2; t^3]; % cubic polynomials
    theta_d = [a1'*vec_t; a2'*vec_t];
    
    %save trajectory joint angles for each time
    trajectory(i,:) = theta_d';
end

if plot_inverseDC
    %% TODO: GENERATE TRAJECTORY USING TwoLinkArmTraj matlab file.
    %% Implement the inverse dynamic control
    options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
    [T,X] = ode45(@(t,x) inverseDC(t, x, a1, a2), [0 tf], x0, options);
    
    %plotting
    plotTrajectories(1, 'Inverse Dynamic', time, trajectory(:,1), T, X(:,1));
    plotTrajectories(2, 'Inverse Dynamic', time, trajectory(:,2), T, X(:,2));
    
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
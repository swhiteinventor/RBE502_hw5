% Notations: For a given variable, x, dx is its time derivative, ddx is
% 2nd-order derivative.
clc
clear all;
close all;

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
plot_lyapunov = true;
plot_passivity = false;

%% trajectory generation for plotting

%set time matrix for plotting
t = 0:0.1:tf

%initialize the trajectory maxtrix
trajectory = zeros(length(t),2);

for i = t
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
    figure('Name','Theta_1 under inverse dynamic control');
    plot(T, X(:,1),'r-');
    title('Theta 1 under inverse dynamic control')
    xlabel('Time (s)')
    ylabel('Theta 1 (radians)')
    hold on
    
    figure('Name','Theta_2 under inverse dynamic control');
    plot(T, X(:,2),'r--');
    title('Theta 2 under inverse dynamic control')
    xlabel('Time (s)')
    ylabel('Theta 2 (radians)')
    hold on
end
if plot_lyapunov
    %% Implement the lyapunov-based control
    options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
    [T,X] = ode45(@(t,x) lyapunovCtrl(t, x, a1, a2),[0 tf],x0, options);
    
    %plotting
    figure('Name','Theta_1 under lyapunov-based control');
    plot(T, X(:,1),'r-');
    title('Theta 1 under lyapunov-based control')
    xlabel('Time (s)')
    ylabel('Theta 1 (radians)')
    hold on
    
    figure('Name','Theta_2 under lyapunov-based control');
    plot(T, X(:,2),'r--');
    title('Theta 2 under lyapunov-based control')
    xlabel('Time (s)')
    ylabel('Theta 2 (radians)')
    hold on
end

if plot_passivity
    %% Implement the passivity-based control
    %options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
    [T,X] = ode45(@(t,x) passivityCtrl(t,x, a1, a2),[0 tf],x0, options);
    
    %plotting
    figure('Name','Theta_1 under passivity-based control');
    plot(T, X(:,1),'r-');
    title('Theta 1 under passivity-based control')
    xlabel('Time (s)')
    ylabel('Theta 1 (radians)')
    hold on
    
    figure('Name','Theta_2 under passivity-based control');
    plot(T, X(:,2),'r--');
    title('Theta 2 under passivity-based control')
    xlabel('Time (s)')
    ylabel('Theta 2 (radians)')
    hold on
end
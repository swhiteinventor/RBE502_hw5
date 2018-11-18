1% Notations: For a given variable, x, dx is its time derivative, ddx is
% 2nd-order derivative.
clc
clear all;
close all;
% the following parameters for the arm
I1=10;  I2 = 10; m1=5; r1=.5; m2=5; r2=.5; l1=1; l2=1;
g=9.8;

% we compute the parameters in the dynamic model
a = I1+I2+m1*r1^2+ m2*(l1^2+ r2^2);
b = m2*l1*r2;
d = I2+ m2*r2^2;

% initial condition % feel free to change
x0 = [-0.5,0.2,0.1,0.1];
xf = [0, 0, 0, 0];

%final time
tf = 100;

%no figure?
nofigure = 1;

%compute the cubic polynomial coefficients
a1 = TwoLinkArmTraj(x0(1), x0(3), xf(1), xf(3), tf, nofigure);
a2 = TwoLinkArmTraj(x0(2), x0(4), xf(2), xf(4), tf, nofigure);

% the options for ode
param = {g, x0, xf, a1, a2, m1, m2, I1, I2, l1, l2, r1, r2};

inverseDC = true;
lyapunov = false;
passivity = false;

if inverseDC
    %% TODO: GENERATE TRAJECTORY USING TwoLinkArmTraj matlab file.
    %% Implement the inverse dynamic control
    options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
    [T,X] = ode45(@(t,x) inverseDC(t, x, param),[0 tf],x0, options);

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
if lyapunov
    %% Implement the lyapunov-based control
    options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
    [T,X] = ode45(@(t,x) lyapunovCtrl(t, x, param),[0 tf],x0, options);

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

if passivity
    %% Implement the passivity-based control
    %options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
    [T,X] = ode45(@(t,x) passivityCtrl(t,x),[0 tf],x0, options);

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
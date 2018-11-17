function [ dx ] = inverseDC( t, x, param )
%param should include: a1, a2 (trajectory parameters),
%m1,m2,I1,I2,l1,l2,r1,r2 (system parameters);

%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    [g, x0, xf, a1, a2, m1, m2, I1, I2, l1, l2, r1, r2] = deal(param{:});

    % note x is in the form of q_1, q_2,dot q_1, dot q_2
    vec_t = [1; t; t^2; t^3]; % cubic polynomials
    theta_d= [a1'*vec_t; a2'*vec_t];
    %ref = [ref,theta_d];
    % compute the velocity and acceleration in both theta 1 and theta2.
    a1_vel = [a1(2), 2*a1(3), 3*a1(4), 0];
    a1_acc = [2*a1(3), 6*a1(4),0,0 ];
    a2_vel = [a2(2), 2*a2(3), 3*a2(4), 0];
    a2_acc = [2*a2(3), 6*a2(4),0,0 ];

    % compute the desired trajectory (assuming 3rd order polynomials for trajectories)
    dtheta_d =[a1_vel*vec_t; a2_vel* vec_t];
    ddtheta_d =[a1_acc*vec_t; a2_acc* vec_t];
    theta= x(1:2,1);
    dtheta= x(3:4,1);

    a = I1+I2*t+m1*r1^2+ m2*(l1^2+ r2^2);
    b = m2*l1*r2;
    d = I2+ m2*r2^2;
    
    % the actual dynamic model of the system:
    Mmat = [a+2*b*cos(x(2)), d+b*cos(x(2));  d+b*cos(x(2)), d];
    Cmat = [-b*sin(x(2))*x(4), -b*sin(x(2))*(x(3)+x(4)); b*sin(x(2))*x(3),0];
    Gmat =  [m1*g*r1*cos(x(1))+m2*g*(l1*cos(x(1))+r2*cos(x(1)+x(2)));
        m2*g*r2*cos(x(1)+x(2))];
    invM = inv(Mmat);
    invMC = invM*Cmat;

    % TODO: compute the control input for the system, which
    % should provide the torques
    % u = 0
    % use the computed torque and state space model to compute
    % the increment in state vector.
    %TODO: compute dx = f(x,u) hint dx(1)=x(3); dx(2)= x(4); the rest
    %of which depends on the dynamic model of the robot.

    %% outputs
%initialize output of function
dx = zeros(4,1);

%% controller
%gain constants, positive definite diagonal matrices
kp = [150 0
      0 150];
kd = [100 0,
      0  100];

%calculate error
e = theta - dtheta;
e_dot = dtheta_d - ddtheta_d;

%{
e = [(x(1) - xf(1)),
     (x(2) - xf(2))];
 
e_dot = [(x(3)- xf(3)),
         (x(4)- xf(4))];
%}

%controller
u = zeros(2,1);
u = -kp*e - kd*e_dot;

%calculate impact
q_dot_dot = zeros(2,1);
q_dot_dot = invM*u - invMC*u;

%final outputs
dx(1) = x(3,1);
dx(2) = x(4,1);
dx(3) = q_dot_dot(1);
dx(4) = q_dot_dot(2);
end


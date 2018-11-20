%% Creates the ODE function for a two link planar arm tracking a cubic polynomial trajectory by lyapunov-based control
%For an n-link serial robot, the format for calculating the dx is:
%dx = Ax + Bu
%u = -kp*e -kd*e_dot
%A = [0 In, 0  0]
%B = [0, In]

function [ dx ] = lyapunovCtrl( t, x, param)
%% inputs
[g, x0, xf, a1, a2, m1, m2, I1, I2, l1, l2, r1, r2] = deal(param{:});

%% trajectory generation

% note x is in the form of q_1, q_2,dot q_1, dot q_2
vec_t = [1; t; t^2; t^3]; % cubic polynomials
theta_d = [a1'*vec_t; a2'*vec_t];
%ref = [ref,theta_d];
% compute the velocity and acceleration in both theta 1 and theta2.
a1_vel = [a1(2), 2*a1(3), 3*a1(4), 0];
a1_acc = [2*a1(3), 6*a1(4),0,0 ];
a2_vel = [a2(2), 2*a2(3), 3*a2(4), 0];
a2_acc = [2*a2(3), 6*a2(4),0,0 ];

% compute the desired trajectory (assuming 3rd order polynomials for trajectories)
dtheta_d =[a1_vel*vec_t; a2_vel* vec_t];
ddtheta_d =[a1_acc*vec_t; a2_acc* vec_t];
theta = x(1:2,1);
dtheta = x(3:4,1);

%% planar arm dynamics

% we compute the parameters in the dynamic model
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

%% outputs
%initialize output of function
dx = zeros(4,1);

%% ilyapunov-based ccontroller

%gain constant, positive definite matrix
kd = [5 0; ...
      0 5];
  
%constant, positive definite square matrices
capital_lambda = [5 0; ...
                  0 5];

%calculate tracking error
e = theta - theta_d;
e_dot = dtheta - dtheta_d;
              
%si calculation
si = dtheta_d - capital_lambda*e;
si_dot = ;
si_dot_dot = ;

%sigma calculation
I = eye(2,2);
sigma = I*e_dot + capital_lambda*e;

%controller
u = zeros(2,1);
u = Mmat*diff(diff(si)) + C*diff(si) + Gmat - kd*sigma;

%calculate impact
q_dot_dot = zeros(2,1);
q_dot_dot = - invMC(q_dot - dif(si)) - invM*kd*sigma + dif(dif(si));

%final outputs
dx(1) = x(3,1);
dx(2) = x(4,1);
dx(3) = q_dot_dot(1);
dx(4) = q_dot_dot(2);

end


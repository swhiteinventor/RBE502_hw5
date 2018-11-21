%% inverseDC
% Creates the ODE function for a two link planar arm tracking a cubic
% polynomial trajectory by inverse dynamic control.

function [ dx ] = inverseDC( t, x, a1, a2)

%% Constants and Variables:
% Set the parameters for the arm:
I1=10;  I2 = 10; m1=5; r1=.5; m2=5; r2=.5; l1=1; l2=1;
g=9.8;
%%
% Calculate the parameters in the dynamic model:
a = I1+I2+m1*r1^2+ m2*(l1^2+ r2^2);
b = m2*l1*r2;
d = I2+ m2*r2^2;

%% Trajectory Generation:
% Note _x_ is in the form of _q1_, _q2_, _q1_dot_, _q2_dot_:
% 
% Cubic polynomials:
vec_t = [1; t; t^2; t^3]; 
theta_d = [a1'*vec_t; a2'*vec_t];
%% 
% Calculate the velocity and acceleration in both _theta 1_ and _theta 2_:
a1_vel = [a1(2), 2*a1(3), 3*a1(4), 0];
a1_acc = [2*a1(3), 6*a1(4),0,0 ];
a2_vel = [a2(2), 2*a2(3), 3*a2(4), 0];
a2_acc = [2*a2(3), 6*a2(4),0,0 ];
%% 
% Calculate the desired trajectory (assuming 3rd order polynomials for
% trajectories):
dtheta_d =[a1_vel*vec_t; a2_vel* vec_t];
ddtheta_d =[a1_acc*vec_t; a2_acc* vec_t];
theta = x(1:2,1);
theta_dot = x(3:4,1);

%% Planar Arm Dynamics:
% Calculate the parameters in the dynamic model:
a = I1+I2*t+m1*r1^2+ m2*(l1^2+ r2^2);
b = m2*l1*r2;
d = I2+ m2*r2^2;
%% 
% Calculate the actual dynamic model of the system:
Mmat = [a+2*b*cos(x(2)), d+b*cos(x(2));  d+b*cos(x(2)), d];
Cmat = [-b*sin(x(2))*x(4), -b*sin(x(2))*(x(3)+x(4)); b*sin(x(2))*x(3),0];
Gmat = [m1*g*r1*cos(x(1))+m2*g*(l1*cos(x(1))+r2*cos(x(1)+x(2)));
    m2*g*r2*cos(x(1)+x(2))];
invM = inv(Mmat);
invMC = invM*Cmat;

%% Inverse Dynamic Ccontroller:
% Set the _kp_ and _kd_ gain constants (positive definite diagonal matrices):
kp = [150 0
    0 150];
kd = [100 0,
    0  100];
%% 
% Calculate the tracking errors, _e_ and _e_dot_:
e = theta - theta_d;
e_dot = theta_dot - dtheta_d;
%% 
% Calculate the _aq_ matrix:
aq_desired = ddtheta_d;
aq = aq_desired - kp*e - kd*e_dot;
%% 
% Calculate the controller, _u_:
u = zeros(2,1);
u = Mmat*aq + Cmat*theta_dot + Gmat;

%% 
% Calculate the acceleration values:
theta_dot_dot = invM*( u - Cmat*theta_dot - Gmat);

%% Outputs
% Initialize the output of the function, _dx_:
dx = zeros(4,1);
%% 
% Set the final outputs:
dx(1) = x(3,1);
dx(2) = x(4,1);
dx(3) = theta_dot_dot(1);
dx(4) = theta_dot_dot(2);
end


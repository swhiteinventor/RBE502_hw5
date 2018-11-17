%% Two Link Arm Trajectory Generater
% Trajectory planning using cubic polynomial
function [a] = TwoLinkArmTraj(theta10, dtheta10, theta1f, dtheta1f, tf, nofigure)
% Input: initial and final positions and velocities.
% Output: a vector a such that the cubic polynomial is
% a(1) + a(2)t + a(3)t^2 + a(4)t^3;

M= [1 0 0 0;
    0 1 0 0;
    1 tf tf^2 tf^3;
    0 1 2*tf 3*tf^2];
b=[theta10; dtheta10;theta1f; dtheta1f];
a=M\b;
t=0:0.01:tf;

if nofigure==1
    return
else
    
    figure('Name','Position (degree)');
    plot(t,a(1)+a(2)*t+ a(3)*t.^2+a(4)*t.^3,'LineWidth',3);
    title('Position (degree)')
    grid
    
    figure('Name','Velocity (degree/s)');
    plot(t,a(2)*t+ 2*a(3)*t +3*a(4)*t.^2,'LineWidth',3);
    title('Velocity (degree/s)')
    grid
    
    figure('Name','Acceleration (degree/s^2)');
    plot(t, 2*a(3) +6*a(4)*t,'LineWidth',3);
    title('Acceleration (degree/s^2)')
    grid
end
end




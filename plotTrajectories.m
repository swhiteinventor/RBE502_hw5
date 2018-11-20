%% plotTrajectories
% This function plots two input trajectories with specific formatting based
% on the input angle number and the input control method.

function plotTrajectories(theta, control, time, trajectory, T, X)

%creates name of plot
name = sprintf('Angle %d Under %s Control', theta, control);

%creates figure
figure('Name',name);
hold on

%plots trajectories
plot(time, trajectory, ...
    'b', ...
    'DisplayName', sprintf('Desired Trajectory: Theta %d', theta));
plot(T, X, ...
    'r-', ...
    'DisplayName', sprintf('Tracked Trajectory: Theta %d', theta));

%sets lables
title(name);
xlabel('Time (s)');
ylabel(sprintf('Theta %d (radians)', theta));
legend;

end
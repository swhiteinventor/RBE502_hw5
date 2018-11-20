%% plotTrajectories
% This function plots two input trajectories with specific formatting based
% on the input angle number and the input control method.

function plotTrajectories(angle, control, time, trajectory, T, X)

%creates name of plot
name = sprintf('Angle %d Under %s Control', angle, control);

%creates figure
figure('Name',name);
hold on

%plots trajectories
plot(time, trajectory, ...
    'b', ...
    'DisplayName', sprintf('Desired Trajectory: Angle %d', angle);
plot(T, X, ...
    'r-', ...
    'DisplayName', sprintf('Tracked Trajectory: Angle %d', angle);

%sets lables
title(name);
xlabel('Time (s)');
ylabel(sprintf('Angle %s (radians)', angle));
legend;

end
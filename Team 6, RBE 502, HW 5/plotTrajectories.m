%% plotTrajectories
% This function plots two input trajectories with specific formatting based
% on the input angle number and the input control method.

function plotTrajectories(theta, control, time, trajectory, T, X, T_error, X_error)
%% 
% Creates name of the plot:
name = sprintf('Theta %d Under %s Control', theta, control);
%% 
% Creates the figure:
figure('Name',name);
hold on 
% Plot the trajectories:
plot(time, trajectory, ...
    'b', ...
    'LineWidth', 2, ... 
    'DisplayName', sprintf('Desired Trajectory: Theta %d', theta));
plot(T, X, ...
    'g--', ...
    'LineWidth', 2, ...
    'DisplayName', sprintf('Tracked Trajectory: Theta %d', theta));
plot(T_error, X_error, ...
    'r:', ...
    'LineWidth', 2, ...
    'DisplayName', sprintf('Tracked Trajectory with Initial Error: Theta %d', theta)); 
% Sets lables:
title(name);
xlabel('Time (s)');
ylabel(sprintf('Theta %d (radians)', theta));
legend('location', 'northwest');

end
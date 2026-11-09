function plot_environment(ax, env, robot)
% Disegna ambiente, ostacoli, configurazione iniziale e finale.
axes(ax); hold(ax, 'on'); axis(ax, 'equal');
xlim(ax, env.xlim); ylim(ax, env.ylim);
grid(ax, 'on'); box(ax, 'on');
xlabel(ax, 'x [m]'); ylabel(ax, 'y [m]');

th = linspace(0, 2*pi, 100);
for i = 1:size(env.circ, 1)
    fill(ax, env.circ(i, 1) + env.circ(i, 3) * cos(th), ...
             env.circ(i, 2) + env.circ(i, 3) * sin(th), [0.6 0.6 0.6], 'EdgeColor', 'k');
end
for i = 1:numel(env.poly)
    fill(ax, env.poly{i}(:, 1), env.poly{i}(:, 2), [0.6 0.6 0.6], 'EdgeColor', 'k');
end

L = 2 * robot.rb;
quiver(ax, env.q0(1), env.q0(2), L * cos(env.q0(3)), L * sin(env.q0(3)), 0, ...
    'g', 'LineWidth', 2, 'MaxHeadSize', 2);
quiver(ax, env.qf(1), env.qf(2), L * cos(env.qf(3)), L * sin(env.qf(3)), 0, ...
    'r', 'LineWidth', 2, 'MaxHeadSize', 2);
plot(ax, env.q0(1), env.q0(2), 'go', 'MarkerFaceColor', 'g');
plot(ax, env.qf(1), env.qf(2), 'ro', 'MarkerFaceColor', 'r');
end

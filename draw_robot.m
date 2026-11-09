function hnd = draw_robot(ax, q, robot, hnd)
% Disegna il robot (corpo, direzione, ruote); se hnd e' dato aggiorna i grafici esistenti
th = linspace(0, 2*pi, 40);
body = [robot.rb * cos(th); robot.rb * sin(th)];
head = [0 robot.rb; 0 0];
lw = 0.12;  ww = 0.04;
wheel = [-lw/2 lw/2 lw/2 -lw/2; -ww/2 -ww/2 ww/2 ww/2];
wR = wheel + [0; -robot.d / 2];
wL = wheel + [0;  robot.d / 2];

R = [cos(q(3)) -sin(q(3)); sin(q(3)) cos(q(3))];
tr = @(P) R * P + q(1:2);
B = tr(body);  H = tr(head);  WR = tr(wR);  WL = tr(wL);

if nargin < 4 || isempty(hnd)
    hnd.body = fill(ax, B(1, :), B(2, :), [0.3 0.6 1.0], 'FaceAlpha', 0.5, 'EdgeColor', 'b');
    hnd.head = plot(ax, H(1, :), H(2, :), 'b-', 'LineWidth', 2);
    hnd.wR = fill(ax, WR(1, :), WR(2, :), 'k');
    hnd.wL = fill(ax, WL(1, :), WL(2, :), 'k');
else
    set(hnd.body, 'XData', B(1, :), 'YData', B(2, :));
    set(hnd.head, 'XData', H(1, :), 'YData', H(2, :));
    set(hnd.wR, 'XData', WR(1, :), 'YData', WR(2, :));
    set(hnd.wL, 'XData', WL(1, :), 'YData', WL(2, :));
end
end

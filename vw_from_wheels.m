function [v, w] = vw_from_wheels(wR, wL, robot)
% Cinematica diretta del robot differenziale:
%   v = r/2 (wR + wL),   w = r/d (wR - wL)
% robot.rw = raggio delle ruote [m], robot.d = interasse [m].
v = robot.rw / 2 * (wR + wL);
w = robot.rw / robot.d * (wR - wL);
end

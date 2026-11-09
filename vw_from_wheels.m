function [v, w] = vw_from_wheels(wR, wL, robot)
% Cinematica diretta: v = r/2 (wR + wL), w = r/d (wR - wL)
v = robot.rw / 2 * (wR + wL);
w = robot.rw / robot.d * (wR - wL);
end

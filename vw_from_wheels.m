function [v, omega] = vw_from_wheels(omega_R, omega_L, robot)
% Cinematica diretta: v = r/2 (omega_R + omega_L),  omega = r/d (omega_R - omega_L)
v     = robot.r / 2 * (omega_R + omega_L);
omega = robot.r / robot.d * (omega_R - omega_L);
end

function [omega_R, omega_L, saturato] = wheels_from_vw(v, omega, robot)
% Cinematica inversa: omega_R = (v + omega d/2) / r,  omega_L = (v - omega d/2) / r
% Se una ruota supera omega_max le scalo entrambe, cosi' la curvatura non cambia
omega_R = (v + omega * robot.d / 2) / robot.r;
omega_L = (v - omega * robot.d / 2) / robot.r;
m = max(abs([omega_R, omega_L]));
saturato = m > robot.omega_max;
if saturato
    omega_R = omega_R * robot.omega_max / m;
    omega_L = omega_L * robot.omega_max / m;
end
end

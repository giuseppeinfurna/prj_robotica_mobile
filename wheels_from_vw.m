function [wR, wL, saturato] = wheels_from_vw(v, w, robot)
% Cinematica inversa: wR = (v + w d/2) / r, wL = (v - w d/2) / r
% Se una ruota supera wmax le scalo entrambe, cosi' la curvatura non cambia
wR = (v + w * robot.d / 2) / robot.rw;
wL = (v - w * robot.d / 2) / robot.rw;
m = max(abs([wR, wL]));
saturato = m > robot.wmax;
if saturato
    wR = wR * robot.wmax / m;
    wL = wL * robot.wmax / m;
end
end

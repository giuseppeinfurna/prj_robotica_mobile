function a = wrap_angle(a)
% Porta l'angolo in (-pi, pi]
a = atan2(sin(a), cos(a));
end

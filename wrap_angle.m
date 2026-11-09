function a = wrap_angle(a)
% Riporta l'angolo (o il vettore di angoli) nell'intervallo (-pi, pi].
a = atan2(sin(a), cos(a));
end

function d = point_segment_dist(P, A, B)
% Distanza di ciascun punto di P (N x 2) dal segmento AB (A, B: 1 x 2).
AB = B - A;
L2 = AB * AB';
if L2 == 0
    t = zeros(size(P, 1), 1);
else
    t = ((P(:, 1) - A(1)) * AB(1) + (P(:, 2) - A(2)) * AB(2)) / L2;
    t = min(max(t, 0), 1);
end
Hx = A(1) + t * AB(1);
Hy = A(2) + t * AB(2);
d = hypot(P(:, 1) - Hx, P(:, 2) - Hy);
end

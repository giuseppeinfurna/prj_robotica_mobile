function d = dist_to_obstacles(P, env)
% Distanza minima dei punti P (N x 2) dagli ostacoli dell'ambiente env.
% Vale 0 per i punti interni a un ostacolo.
%   env.circ : M x 3, righe [cx cy r]
%   env.poly : cell array, ogni cella e' una matrice K x 2 di vertici
d = inf(size(P, 1), 1);

for i = 1:size(env.circ, 1)
    di = max(hypot(P(:, 1) - env.circ(i, 1), P(:, 2) - env.circ(i, 2)) - env.circ(i, 3), 0);
    d = min(d, di);
end

for i = 1:numel(env.poly)
    V = env.poly{i};
    K = size(V, 1);
    di = inf(size(P, 1), 1);
    for j = 1:K
        di = min(di, point_segment_dist(P, V(j, :), V(mod(j, K) + 1, :)));
    end
    di(inpolygon(P(:, 1), P(:, 2), V(:, 1), V(:, 2))) = 0;
    d = min(d, di);
end
end

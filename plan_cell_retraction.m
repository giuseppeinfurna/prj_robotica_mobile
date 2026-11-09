function plan = plan_cell_retraction(env, robot, h)
% Pianificazione con cell decomposition e retraction.
%   1) decomposizione in celle quadrate di lato h, libere o occupate
%      (ostacoli ingranditi del raggio del robot piu' un margine);
%   2) grafo di adiacenza delle celle libere (8-connesso);
%   3) retraction: ogni cella libera viene riportata sulla "dorsale" della
%      mappa di clearance (luogo dei punti piu' lontani dagli ostacoli);
%   4) Dijkstra sul grafo ridotto alla dorsale.
% Output: struttura plan con griglia, clearance, dorsale, grafo e waypoint.

infl = robot.rb + robot.margin;
nx = round((env.xlim(2) - env.xlim(1)) / h);
ny = round((env.ylim(2) - env.ylim(1)) / h);
xc = env.xlim(1) + ((1:nx) - 0.5) * h;
yc = env.ylim(1) + ((1:ny) - 0.5) * h;
[X, Y] = meshgrid(xc, yc);                 % righe = y, colonne = x

%% 1) Celle libere e occupate
dobs = dist_to_obstacles([X(:), Y(:)], env);
occ  = reshape(dobs < infl, ny, nx);
free = ~occ;

%% 2) Grafo di adiacenza delle celle libere (pesi = distanza tra i centri)
A = build_adjacency(free, h);

%% 3) Mappa di clearance e retraction sulla dorsale
ox = X(occ);  oy = Y(occ);
C = zeros(ny, nx);
for k = find(free)'
    dk = min([X(k) - env.xlim(1), env.xlim(2) - X(k), Y(k) - env.ylim(1), env.ylim(2) - Y(k)]);
    if ~isempty(ox)
        dk = min(dk, sqrt(min((ox - X(k)).^2 + (oy - Y(k)).^2)));
    end
    C(k) = dk;
end

Cp = zeros(ny + 2, nx + 2);
Cp(2:end-1, 2:end-1) = C;
c  = C;
E  = Cp(2:end-1, 3:end);   W  = Cp(2:end-1, 1:end-2);
N  = Cp(1:end-2, 2:end-1); S  = Cp(3:end, 2:end-1);
NE = Cp(1:end-2, 3:end);   SW = Cp(3:end, 1:end-2);
NW = Cp(1:end-2, 1:end-2); SE = Cp(3:end, 3:end);
% una cella e' sulla dorsale se e' un massimo locale di clearance lungo almeno una direzione
ridge = free & ( (c > E  & c >= W ) | (c >= E  & c > W ) | ...
                 (c > N  & c >= S ) | (c >= N  & c > S ) | ...
                 (c > NE & c >= SW) | (c >= NE & c > SW) | ...
                 (c > NW & c >= SE) | (c >= NW & c > SE) );

% celle di partenza e arrivo e loro retraction sulla dorsale
cell0 = point_to_cell(env.q0(1:2), env, h, nx, ny);
cellf = point_to_cell(env.qf(1:2), env, h, nx, ny);
if ~free(cell0) || ~free(cellf)
    error('plan_cell_retraction:Occupied', 'Configurazione iniziale o finale in una cella occupata.');
end
up0 = retract(cell0, C, free, ridge, nx, ny);
upf = retract(cellf, C, free, ridge, nx, ny);

%% 4) Dijkstra sul grafo ridotto alla dorsale
skel = find(ridge);
Ws = A(skel, skel);
s = find(skel == up0(end));
t = find(skel == upf(end));
npath = dijkstra_sparse(Ws, s, t);
if isempty(npath)
    error('plan_cell_retraction:NoPath', 'Nessun percorso sulla dorsale tra start e goal.');
end
cells = [up0(:); skel(npath(2:end)); flipud(upf(:))];
cells = cells([true; diff(cells) ~= 0]);

pts = [X(cells), Y(cells)];
raw = [env.q0(1:2)'; pts; env.qf(1:2)'];
raw = raw([true; any(diff(raw) ~= 0, 2)], :);

%% 5) Waypoint essenziali (si eliminano i punti allineati se il tratto e' libero)
idx = rdp(raw, 1, size(raw, 1), 1.5 * h, env, robot.rb + 0.5 * robot.margin, h);
wp = raw(idx, :);

plan.h = h;  plan.xc = xc;  plan.yc = yc;
plan.occ = occ;  plan.free = free;  plan.clearance = C;
plan.ridge = ridge;  plan.adj = A;
plan.raw = raw;  plan.waypoints = wp;
plan.infl = infl;
plan.n_free = nnz(free);
plan.n_occ = nnz(occ);
plan.n_edges = nnz(A) / 2;
plan.n_ridge = nnz(ridge);
plan.length = sum(hypot(diff(wp(:, 1)), diff(wp(:, 2))));
end

%% ---------------------------------------------------------------
function k = point_to_cell(p, env, h, nx, ny)
col = min(max(floor((p(1) - env.xlim(1)) / h) + 1, 1), nx);
row = min(max(floor((p(2) - env.ylim(1)) / h) + 1, 1), ny);
k = row + (col - 1) * ny;
end

function A = build_adjacency(free, h)
% Matrice sparsa simmetrica: A(i,j) = distanza tra i centri se le celle sono adiacenti.
[ny, nx] = size(free);
[R, Cc] = ndgrid(1:ny, 1:nx);
ii = [];  jj = [];  ww = [];
for dr = -1:1
    for dc = -1:1
        if dr == 0 && dc == 0, continue; end
        r2 = R + dr;  c2 = Cc + dc;
        ok = free & r2 >= 1 & r2 <= ny & c2 >= 1 & c2 <= nx;
        src = find(ok);
        dst = r2(src) + (c2(src) - 1) * ny;
        keep = free(dst);
        if dr ~= 0 && dc ~= 0       % niente scorciatoie in diagonale tra due ostacoli
            side1 = R(src) + dr + (Cc(src) - 1) * ny;
            side2 = R(src) + (Cc(src) + dc - 1) * ny;
            keep = keep & free(side1) & free(side2);
        end
        src = src(keep);  dst = dst(keep);
        ii = [ii; src];  jj = [jj; dst];  ww = [ww; h * hypot(dr, dc) * ones(numel(src), 1)];
    end
end
A = sparse(ii, jj, ww, ny * nx, ny * nx);
end

function path = retract(k, C, free, ridge, nx, ny)
% Salita a gradiente sulla clearance fino alla dorsale (mappa di retraction).
path = k;
for it = 1:nx * ny
    if ridge(k), return; end
    r = mod(k - 1, ny) + 1;  c = floor((k - 1) / ny) + 1;
    best = k;  bestC = C(k);
    for dr = -1:1
        for dc = -1:1
            r2 = r + dr;  c2 = c + dc;
            if (dr == 0 && dc == 0) || r2 < 1 || r2 > ny || c2 < 1 || c2 > nx, continue; end
            k2 = r2 + (c2 - 1) * ny;
            if free(k2) && C(k2) > bestC
                best = k2;  bestC = C(k2);
            end
        end
    end
    if best == k, return; end
    k = best;
    path(end + 1, 1) = k; %#ok<AGROW>
end
end

function path = dijkstra_sparse(W, s, t)
% Dijkstra con pesi non negativi su matrice sparsa simmetrica W.
n = size(W, 1);
dist = inf(n, 1);  prev = zeros(n, 1);  done = false(n, 1);
dist(s) = 0;
for it = 1:n
    tmp = dist;  tmp(done) = inf;
    [dmin, u] = min(tmp);
    if isinf(dmin) || u == t, break; end
    done(u) = true;
    [nb, ~, w] = find(W(:, u));
    for q = 1:numel(nb)
        v = nb(q);
        if ~done(v) && dist(u) + w(q) < dist(v)
            dist(v) = dist(u) + w(q);
            prev(v) = u;
        end
    end
end
if isinf(dist(t))
    path = [];
    return;
end
path = t;
while path(1) ~= s
    path = [prev(path(1)); path];
end
end

function idx = rdp(P, i, j, tol, env, dmin, h)
% Ramer-Douglas-Peucker: un punto viene eliminato solo se resta entro tol
% dal segmento e il segmento stesso e' collision-free.
if j <= i + 1
    idx = [i j];
    return;
end
d = point_segment_dist(P(i+1:j-1, :), P(i, :), P(j, :));
[dm, km] = max(d);
if dm <= tol && segment_free(P(i, :), P(j, :), env, dmin, h)
    idx = [i j];
else
    k = i + km;
    a = rdp(P, i, k, tol, env, dmin, h);
    b = rdp(P, k, j, tol, env, dmin, h);
    idx = [a, b(2:end)];
end
end

function ok = segment_free(A, B, env, dmin, h)
n = max(2, ceil(norm(B - A) / (h / 2)) + 1);
s = linspace(0, 1, n)';
Q = A + s * (B - A);
ok = all(dist_to_obstacles(Q, env) >= dmin);
end

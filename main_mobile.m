%% Robot mobile differenziale: pianificazione e controllo
% Cell decomposition + retraction con Dijkstra, poi controllo in coordinate polari
clear; clc; close all;

%% Parametri del robot e del controllo
robot.rw     = 0.05;      % raggio delle ruote [m]
robot.d      = 0.30;      % interasse [m]
robot.rb     = 0.20;      % raggio del corpo [m]
robot.margin = 0.15;      % margine di sicurezza [m]
robot.wmax   = 15;        % velocita' massima delle ruote [rad/s]

ctrl.k_rho     = 1.5;     % guadagno su rho   (k_rho > 0)
ctrl.k_alpha   = 4.0;     % guadagno su alpha (k_alpha > k_rho)
ctrl.tol_wp    = 0.30;    % tolleranza sui waypoint intermedi [m]
ctrl.tol_final = 0.03;    % tolleranza sul waypoint finale [m]
ctrl.tol_theta = 0.02;    % tolleranza sull'orientamento finale [rad]
ctrl.dt        = 0.01;    % passo di integrazione [s]
ctrl.Tmax      = 120;     % tempo massimo di simulazione [s]

h = 0.1;                  % lato delle celle [m]
salva_gif = true;         % salva anche results/animazione.gif

if ~exist('results', 'dir'), mkdir('results'); end

%% 1) Modello differenziale: verifica di (wR, wL) <-> (v, w)
v_t = 0.4;  w_t = -0.8;
[wR_t, wL_t] = wheels_from_vw(v_t, w_t, robot);
[v_c, w_c] = vw_from_wheels(wR_t, wL_t, robot);
fprintf('Modello: v = %.2f m/s, w = %.2f rad/s -> wR = %.3f, wL = %.3f rad/s\n', v_t, w_t, wR_t, wL_t);
assert(abs(v_c - v_t) < 1e-12 && abs(w_c - w_t) < 1e-12, 'Cinematica diretta e inversa non coerenti');
[wR_t, wL_t] = wheels_from_vw(1, 0, robot);            % rettilineo
assert(abs(wR_t - wL_t) < 1e-12, 'In moto rettilineo le ruote devono avere la stessa velocita''');
[wR_t, wL_t] = wheels_from_vw(0, 1, robot);            % rotazione sul posto
assert(abs(wR_t + wL_t) < 1e-12, 'In rotazione sul posto le ruote devono essere opposte');
disp('Test del modello superati.');
fprintf('\n');

%% 2) Ambiente
env = build_environment();
d0 = dist_to_obstacles(env.q0(1:2)', env);
df = dist_to_obstacles(env.qf(1:2)', env);
assert(d0 > robot.rb && df > robot.rb, 'Configurazione iniziale o finale in collisione');

figure('Name', 'Ambiente');
plot_environment(gca, env, robot);
title('Ambiente: ostacoli, configurazione iniziale (verde) e finale (rosso)');

%% 3) Pianificazione: cell decomposition con retraction
plan = plan_cell_retraction(env, robot, h);
wp = plan.waypoints;
fprintf('Celle libere: %d, occupate: %d, archi del grafo: %d\n', plan.n_free, plan.n_occ, plan.n_edges);
fprintf('Celle sulla dorsale (retraction): %d\n', plan.n_ridge);
d_retta = norm(env.qf(1:2) - env.q0(1:2));
fprintf('Waypoint: %d, lunghezza del percorso: %.2f m\n', size(wp, 1), plan.length);
fprintf('Distanza in linea retta q0-qf: %.2f m (rapporto percorso/retta: %.2f)\n\n', d_retta, plan.length / d_retta);

% controllo che il percorso sia libero da collisioni anche con il robot
seg = [];
for i = 1:size(wp, 1) - 1
    s = linspace(0, 1, max(2, ceil(norm(wp(i+1, :) - wp(i, :)) / 0.02)))';
    seg = [seg; wp(i, :) + s .* (wp(i+1, :) - wp(i, :))]; %#ok<AGROW>
end
dmin_plan = min(dist_to_obstacles(seg, env));
fprintf('Distanza minima dagli ostacoli lungo il percorso pianificato: %.3f m\n', dmin_plan);
assert(dmin_plan > robot.rb, 'Il percorso pianificato collide con un ostacolo');

figure('Name', 'Pianificazione');
ax = gca;
imagesc(ax, plan.xc, plan.yc, double(plan.occ)); set(ax, 'YDir', 'normal');
colormap(ax, [1 1 1; 0.75 0.75 0.75]); hold(ax, 'on');
[X, Y] = meshgrid(plan.xc, plan.yc);
plot(ax, X(plan.ridge), Y(plan.ridge), '.', 'Color', [0.2 0.5 1], 'MarkerSize', 6);
plot(ax, plan.raw(:, 1), plan.raw(:, 2), 'm-', 'LineWidth', 1);
plot(ax, wp(:, 1), wp(:, 2), 'ro-', 'LineWidth', 1.5, 'MarkerFaceColor', 'r');
axis(ax, 'equal'); xlim(ax, env.xlim); ylim(ax, env.ylim); grid(ax, 'on');
xlabel(ax, 'x [m]'); ylabel(ax, 'y [m]');
title(ax, 'Celle occupate (grigio), dorsale di retraction (blu), percorso e waypoint (rosso)');
legend(ax, {'dorsale', 'percorso su griglia', 'waypoint'}, 'Location', 'southeast');

%% 4) Controllo e simulazione
sim = simulate_diff_drive(env.q0, wp, env.qf(3), robot, ctrl);
t = sim.t;  q = sim.q;
fprintf('Simulazione terminata in %.2f s\n', t(end));

%% 5) Analisi dei risultati
% errore di tracking = distanza dalla poligonale dei waypoint
e_cross = inf(1, numel(t));
for i = 1:size(wp, 1) - 1
    e_cross = min(e_cross, point_segment_dist(q(1:2, :)', wp(i, :), wp(i+1, :))');
end
e_pos = hypot(q(1, :) - env.qf(1), q(2, :) - env.qf(2));
e_th  = abs(wrap_angle(q(3, :) - env.qf(3)));
dmin_exec = min(dist_to_obstacles(q(1:2, :)', env));

fprintf('Errore finale di posizione:    %.4f m\n', e_pos(end));
fprintf('Errore finale di orientamento: %.4f rad\n', e_th(end));
fprintf('Errore di tracking massimo / medio (distanza dal percorso): %.3f / %.3f m\n', max(e_cross), mean(e_cross));
fprintf('Distanza minima dagli ostacoli (traiettoria eseguita): %.3f m (raggio robot %.2f m)\n', dmin_exec, robot.rb);
fprintf('Velocita'' massime: v = %.3f m/s, |w| = %.3f rad/s, |wheel| = %.2f rad/s\n', ...
    max(abs(sim.v)), max(abs(sim.w)), max(abs([sim.wR, sim.wL])));

assert(e_pos(end) < ctrl.tol_final + 1e-6, 'Posizione finale fuori tolleranza');
assert(e_th(end) < ctrl.tol_theta + 1e-6, 'Orientamento finale fuori tolleranza');
assert(dmin_exec > robot.rb, 'La traiettoria eseguita collide con un ostacolo');

fid = fopen(fullfile('results', 'riepilogo.txt'), 'w');
fprintf(fid, 'Celle libere: %d\nCelle occupate: %d\nArchi del grafo: %d\nCelle sulla dorsale: %d\n', ...
    plan.n_free, plan.n_occ, plan.n_edges, plan.n_ridge);
fprintf(fid, 'Waypoint: %d\nLunghezza percorso pianificato: %.3f m\n', size(wp, 1), plan.length);
fprintf(fid, 'Distanza in linea retta q0-qf: %.3f m\nRapporto percorso/retta: %.3f\n', d_retta, plan.length / d_retta);
fprintf(fid, 'Tempo di simulazione: %.2f s\n', t(end));
fprintf(fid, 'Errore finale posizione: %.4f m\nErrore finale orientamento: %.4f rad\n', e_pos(end), e_th(end));
fprintf(fid, 'Errore di tracking max/medio: %.3f / %.3f m\n', max(e_cross), mean(e_cross));
fprintf(fid, 'Distanza minima dagli ostacoli: %.3f m\n', dmin_exec);
fprintf(fid, 'Velocita'' massime: v = %.3f m/s, |w| = %.3f rad/s, |wheel| = %.2f rad/s\n', ...
    max(abs(sim.v)), max(abs(sim.w)), max(abs([sim.wR, sim.wL])));
fclose(fid);

%% 6) Grafici
figure('Name', 'Traiettorie');
plot_environment(gca, env, robot);
hp = plot(wp(:, 1), wp(:, 2), 'r--o', 'LineWidth', 1.2, 'MarkerFaceColor', 'r');
he = plot(q(1, :), q(2, :), 'b-', 'LineWidth', 1.8);
legend([hp he], {'percorso pianificato', 'traiettoria eseguita'}, 'Location', 'southeast');
title('Traiettoria pianificata e traiettoria eseguita');

figure('Name', 'Errore di tracking');
subplot(3, 1, 1);
plot(t, e_cross, 'LineWidth', 1.3); grid on;
ylabel('e_{perc} [m]'); title('Distanza dal percorso pianificato');
subplot(3, 1, 2);
plot(t, sim.rho, 'LineWidth', 1.3); grid on;
ylabel('\rho [m]'); title('Distanza dal waypoint corrente');
subplot(3, 1, 3);
plot(t, rad2deg(wrap_angle(sim.alpha)), 'LineWidth', 1.3); grid on;
xlabel('t [s]'); ylabel('\alpha [deg]'); title('Errore di direzione');

figure('Name', 'Velocita'' delle ruote');
subplot(2, 1, 1);
plot(t, sim.wR, t, sim.wL, 'LineWidth', 1.3); grid on;
yline(robot.wmax, 'k:'); yline(-robot.wmax, 'k:');
ylabel('\omega [rad/s]'); legend('\omega_R', '\omega_L'); title('Velocita'' angolari delle ruote');
subplot(2, 1, 2);
yyaxis left;  plot(t, sim.v, 'LineWidth', 1.3); ylabel('v [m/s]');
yyaxis right; plot(t, sim.w, 'LineWidth', 1.3); ylabel('\omega [rad/s]');
grid on; xlabel('t [s]'); title('Velocita'' lineare e angolare del robot');

%% 7) Animazione
figure('Name', 'Animazione');
ax = gca;
plot_environment(ax, env, robot);
plot(ax, wp(:, 1), wp(:, 2), 'r--o', 'MarkerFaceColor', 'r');
traccia = plot(ax, q(1, 1), q(2, 1), 'b-', 'LineWidth', 1.5);
hr = draw_robot(ax, q(:, 1), robot);
title(ax, 'Animazione del robot differenziale');

step = max(1, round(0.05 / ctrl.dt));        % un fotogramma ogni 0.05 s
frames = [1:step:size(q, 2), size(q, 2)];
gif_file = fullfile('results', 'animazione.gif');
for j = 1:numel(frames)
    k = frames(j);
    set(traccia, 'XData', q(1, 1:k), 'YData', q(2, 1:k));
    draw_robot(ax, q(:, k), robot, hr);
    title(ax, sprintf('Animazione del robot differenziale - t = %.1f s', t(k)));
    drawnow limitrate;
    if salva_gif
        [A, map] = rgb2ind(frame2im(getframe(ax)), 256);
        if j == 1
            imwrite(A, map, gif_file, 'gif', 'LoopCount', Inf, 'DelayTime', 0.05);
        else
            imwrite(A, map, gif_file, 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);
        end
    end
end

%% 8) Salvataggio delle figure in results/
titoli = {'Ambiente', 'Pianificazione', 'Traiettorie', 'Errore di tracking', 'Velocita'' delle ruote', 'Animazione'};
nomi   = {'ambiente', 'pianificazione', 'traiettorie', 'errore_tracking', 'velocita_ruote', 'animazione_fine'};
for i = 1:numel(nomi)
    exportgraphics(findobj('Type', 'figure', 'Name', titoli{i}), fullfile('results', [nomi{i} '.png']), 'Resolution', 150);
end
fprintf('Figure salvate nella cartella results/\n');

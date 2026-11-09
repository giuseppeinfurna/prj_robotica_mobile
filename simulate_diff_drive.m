function sim = simulate_diff_drive(q0, wp, theta_f, robot, ctrl)
% Simulazione del robot differenziale con controllore cinematico a coordinate polari
%   rho   = distanza dal waypoint corrente
%   alpha = errore di direzione rispetto al waypoint
%   v = k_rho * rho * cos(alpha)   (v = 0 se |alpha| > pi/2: il robot ruota sul posto)
%   w = k_alpha * alpha
% Fase 1: inseguimento dei waypoint. Fase 2: allineamento all'orientamento finale.
% (v, w) -> (wR, wL) con saturazione delle ruote; integrazione con Runge-Kutta 4.

dt  = ctrl.dt;
Nmax = ceil(ctrl.Tmax / dt);
nwp = size(wp, 1);

q = q0(:);
k = 2;                       % wp(1) e' la posizione di partenza
phase = 1;

t = zeros(1, Nmax + 1);   Q = zeros(3, Nmax + 1);
V = zeros(1, Nmax + 1);   Wz = zeros(1, Nmax + 1);
Om = zeros(2, Nmax + 1);  rho_log = zeros(1, Nmax + 1);
alpha_log = zeros(1, Nmax + 1);  wp_log = zeros(1, Nmax + 1);
Q(:, 1) = q;
n = 0;
done = false;

while n < Nmax && ~done
    n = n + 1;
    if phase == 1
        tol = ctrl.tol_wp;
        if k == nwp, tol = ctrl.tol_final; end
        dx = wp(k, 1) - q(1);  dy = wp(k, 2) - q(2);
        rho = hypot(dx, dy);
        if rho < tol
            if k == nwp
                phase = 2;
            else
                k = k + 1;
                dx = wp(k, 1) - q(1);  dy = wp(k, 2) - q(2);
                rho = hypot(dx, dy);
            end
        end
    end

    if phase == 1
        alpha = wrap_angle(atan2(dy, dx) - q(3));
        v = min(ctrl.k_rho * rho * max(cos(alpha), 0), robot.wmax * robot.rw);
        w = ctrl.k_alpha * alpha;
    else
        rho = 0;
        alpha = wrap_angle(theta_f - q(3));
        if abs(alpha) < ctrl.tol_theta
            done = true;
        end
        v = 0;
        w = ctrl.k_alpha * alpha;
    end

    [wR, wL] = wheels_from_vw(v, w, robot);
    [v, w] = vw_from_wheels(wR, wL, robot);      % valori effettivamente applicati

    t(n) = (n - 1) * dt;
    V(n) = v;  Wz(n) = w;  Om(:, n) = [wR; wL];
    rho_log(n) = rho;  alpha_log(n) = alpha;  wp_log(n) = k;

    q = rk4_step(q, v, w, dt);
    Q(:, n + 1) = q;
end

if ~done
    error('simulate_diff_drive:Timeout', 'Il robot non ha raggiunto la configurazione finale entro Tmax.');
end

% Ultimo campione: stato finale con ingressi nulli
t(n + 1) = n * dt;
sim.t = t(1:n + 1);
sim.q = Q(:, 1:n + 1);
sim.v = [V(1:n), 0];
sim.w = [Wz(1:n), 0];
sim.wR = [Om(1, 1:n), 0];
sim.wL = [Om(2, 1:n), 0];
sim.rho = [rho_log(1:n), 0];
sim.alpha = [alpha_log(1:n), 0];
sim.wp_idx = [wp_log(1:n), nwp];
end

function q = rk4_step(q, v, w, dt)
f = @(q) [v * cos(q(3)); v * sin(q(3)); w];
k1 = f(q);
k2 = f(q + dt / 2 * k1);
k3 = f(q + dt / 2 * k2);
k4 = f(q + dt * k3);
q = q + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
end

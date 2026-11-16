function sim = simulate_diff_drive(q0, wp, theta_f, robot, ctrl)
% Controllo in coordinate polari verso il waypoint corrente (x_i, y_i):
%   rho = distanza, alpha = errore di direzione
%   v = k_rho * rho * cos(alpha) (0 se |alpha| > pi/2),  omega = k_alpha * alpha
% Fase 1: waypoint, fase 2: allineamento finale. Ruote saturate.
% Integrazione con Eulero esplicito: q_{k+1} = q_k + dt * qdot_k

dt   = ctrl.dt;
Nmax = ceil(ctrl.Tmax / dt);
nwp  = size(wp, 1);

q = q0(:);
i = 2;                       % wp(1) e' la posizione di partenza
fase = 1;

t = zeros(1, Nmax + 1);        Q = zeros(3, Nmax + 1);
V = zeros(1, Nmax + 1);        Omega = zeros(1, Nmax + 1);
Omega_RL = zeros(2, Nmax + 1); Rho = zeros(1, Nmax + 1);
Alpha = zeros(1, Nmax + 1);    I_wp = zeros(1, Nmax + 1);
Q(:, 1) = q;
k = 0;
finito = false;

while k < Nmax && ~finito
    k = k + 1;
    if fase == 1
        tol = ctrl.tol_wp;
        if i == nwp, tol = ctrl.tol_final; end
        dx = wp(i, 1) - q(1);  dy = wp(i, 2) - q(2);
        rho = hypot(dx, dy);
        if rho < tol
            if i == nwp
                fase = 2;
            else
                i = i + 1;
                dx = wp(i, 1) - q(1);  dy = wp(i, 2) - q(2);
                rho = hypot(dx, dy);
            end
        end
    end

    if fase == 1
        alpha = wrap_angle(atan2(dy, dx) - q(3));
        v = min(ctrl.k_rho * rho * max(cos(alpha), 0), robot.omega_max * robot.r);
        omega = ctrl.k_alpha * alpha;
    else
        rho = 0;
        alpha = wrap_angle(theta_f - q(3));
        if abs(alpha) < ctrl.tol_theta
            finito = true;
        end
        v = 0;
        omega = ctrl.k_alpha * alpha;
    end

    [omega_R, omega_L] = wheels_from_vw(v, omega, robot);
    [v, omega] = vw_from_wheels(omega_R, omega_L, robot);   % valori effettivamente applicati

    t(k) = (k - 1) * dt;
    V(k) = v;  Omega(k) = omega;  Omega_RL(:, k) = [omega_R; omega_L];
    Rho(k) = rho;  Alpha(k) = alpha;  I_wp(k) = i;

    % Eulero esplicito sul modello differenziale
    qdot = [v * cos(q(3)); v * sin(q(3)); omega];
    q = q + dt * qdot;
    Q(:, k + 1) = q;
end

if ~finito
    error('simulate_diff_drive:Timeout', 'Il robot non ha raggiunto la configurazione finale entro Tmax.');
end

% ultimo campione con ingressi nulli
t(k + 1) = k * dt;
sim.t = t(1:k + 1);
sim.q = Q(:, 1:k + 1);
sim.v = [V(1:k), 0];
sim.omega = [Omega(1:k), 0];
sim.omega_R = [Omega_RL(1, 1:k), 0];
sim.omega_L = [Omega_RL(2, 1:k), 0];
sim.rho = [Rho(1:k), 0];
sim.alpha = [Alpha(1:k), 0];
sim.wp_idx = [I_wp(1:k), nwp];
end

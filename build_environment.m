function env = build_environment()
% Ambiente di lavoro: piano rettangolare 10 x 10 m con ostacoli circolari
% e poligonali, configurazione iniziale q0 e finale qf = [x y theta].
env.xlim = [0 10];
env.ylim = [0 10];

% Ostacoli circolari [cx cy r]
env.circ = [2.5 6.5 1.0;
            7.5 7.0 1.0;
            7.5 2.5 0.9];

% Ostacoli poligonali (vertici in senso antiorario)
env.poly = { [4.5 3.5; 5.5 3.5; 5.5 8.0; 4.5 8.0], ...   % parete verticale
             [1.5 3.0; 3.5 3.0; 2.5 4.5] };                % triangolo

env.q0 = [1; 1; 0];
env.qf = [9; 9; pi/2];
end

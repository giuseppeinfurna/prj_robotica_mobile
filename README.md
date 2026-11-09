# Pianificazione e controllo di un robot mobile differenziale

Questo progetto MATLAB modella un robot mobile differenziale che si muove nel piano XY tra ostacoli circolari e poligonali. Il percorso è pianificato con cell decomposition e retraction, poi seguito da un controllore cinematico in coordinate polari. Le unità sono quelle del Sistema Internazionale e gli angoli sono in radianti.

## Come eseguirlo

Basta mettere tutti i file `.m` nella stessa cartella, impostarla come cartella corrente in MATLAB e lanciare `main_mobile`. Lo script esegue i test del modello, pianifica, simula, mostra i grafici e l'animazione e salva tutto in `results/`. Da terminale si può usare `matlab -batch "main_mobile"`. Non servono toolbox aggiuntivi. Per non generare la GIF basta impostare `salva_gif = false` all'inizio dello script.

## Modello

Lo stato è `q = [x y theta]'` e gli ingressi sono le velocità angolari delle ruote `wR` e `wL`. Con `r` raggio delle ruote e `d` interasse:

```
v = r/2 (wR + wL)        w = r/d (wR - wL)
xdot = v cos(theta)      ydot = v sin(theta)      thetadot = w
```

`vw_from_wheels` implementa la relazione diretta e `wheels_from_vw` quella inversa. Se una ruota supera la velocità massima, entrambe sono scalate dello stesso fattore, così la curvatura della traiettoria non cambia.

## Come è organizzato il codice

| File | Contenuto |
|---|---|
| `main_mobile.m` | script principale: test, pianificazione, simulazione, grafici, animazione, salvataggio |
| `build_environment.m` | ambiente 10 x 10 m, ostacoli, `q0` e `qf` |
| `dist_to_obstacles.m`, `point_segment_dist.m` | modello geometrico degli ostacoli e distanze |
| `plan_cell_retraction.m` | cell decomposition, grafo di adiacenza, retraction, Dijkstra |
| `simulate_diff_drive.m` | controllore, conversione in velocità delle ruote, integrazione RK4 |
| `wheels_from_vw.m`, `vw_from_wheels.m` | modello differenziale |
| `plot_environment.m`, `draw_robot.m`, `wrap_angle.m` | funzioni di supporto |

## Pianificazione: cell decomposition con retraction

1. L'ambiente è diviso in celle quadrate di lato 0.1 m. Gli ostacoli sono ingranditi del raggio del robot più un margine di sicurezza, quindi il robot può essere trattato come un punto. Una cella il cui centro cade nell'ostacolo ingrandito è occupata, le altre sono libere.
2. Le celle libere formano un grafo di adiacenza a 8 vicini, con peso pari alla distanza tra i centri. Le diagonali tra due celle occupate sono vietate per evitare scorciatoie attraverso gli spigoli.
3. Per ogni cella libera si calcola la clearance, cioè la distanza dall'ostacolo più vicino o dal bordo. La retraction porta ogni cella sulla dorsale della mappa di clearance, formata dai massimi locali, che è il luogo dei punti più lontani dagli ostacoli. Partenza e arrivo sono riportati sulla dorsale con una salita a gradiente della clearance.
4. L'algoritmo di Dijkstra cerca il percorso più breve sul grafo ridotto alla dorsale.
5. I waypoint si ottengono eliminando i punti allineati con Ramer-Douglas-Peucker, ma un punto è eliminato solo se il segmento che lo sostituisce è libero da collisioni.

Se non esiste un percorso la funzione solleva l'errore `plan_cell_retraction:NoPath`.

## Controllo

Per il waypoint corrente si calcolano `rho` (distanza) e `alpha` (errore di direzione):

```
v = k_rho * rho * cos(alpha)  (v = 0 se |alpha| > pi/2)
w = k_alpha * alpha
```

con `k_rho > 0` e `k_alpha > k_rho`. Il termine `cos(alpha)` evita che il robot avanzi mentre è girato dalla parte sbagliata. Per i waypoint intermedi basta una tolleranza di 0.3 m, così il robot non rallenta a ogni punto. Raggiunto l'ultimo waypoint, il robot ruota sul posto fino all'orientamento finale. Le velocità `(v, w)` sono convertite in `(wR, wL)` e l'integrazione è fatta con Runge-Kutta del quarto ordine con passo 0.01 s.

Il modello usato dal controllore coincide con quello simulato: non ci sono slittamenti, rumore o dinamica dei motori.

## Verifiche automatiche

`main_mobile` si ferma con un messaggio chiaro se una di queste condizioni non è vera: cinematica diretta e inversa coerenti (anche per moto rettilineo e rotazione sul posto), partenza e arrivo fuori dagli ostacoli, percorso pianificato collision-free con il robot, posizione e orientamento finali entro tolleranza, traiettoria eseguita senza collisioni.

## Output generati

Ogni esecuzione scrive in `results/` i file seguenti.

| File | Contenuto |
|---|---|
| `ambiente.png` | ostacoli, configurazione iniziale (verde) e finale (rossa) |
| `pianificazione.png` | celle occupate, dorsale di retraction, percorso e waypoint |
| `traiettorie.png` | traiettoria pianificata e traiettoria eseguita |
| `errore_tracking.png` | distanza dal percorso, `rho` e `alpha` nel tempo |
| `velocita_ruote.png` | `wR`, `wL`, `v` e `w` nel tempo |
| `animazione_fine.png`, `animazione.gif` | ultimo fotogramma e animazione del moto |
| `riepilogo.txt` | numeri da citare: celle, lunghezza, tempi, errori, distanza minima |

![Ambiente](results/ambiente.png)

![Pianificazione](results/pianificazione.png)

![Traiettorie](results/traiettorie.png)

![Errore di tracking](results/errore_tracking.png)

![Velocità delle ruote](results/velocita_ruote.png)

![Fine dell'animazione](results/animazione_fine.png)

## Analisi dei risultati

Il codice è stato eseguito in MATLAB e tutti i controlli automatici sono stati superati. Con `v = 0.40 m/s` e `w = -0.80 rad/s` il modello dà `wR = 5.600 rad/s` e `wL = 10.400 rad/s`.

La decomposizione produce 7127 celle libere e 2873 occupate, con 27090 archi nel grafo di adiacenza. La dorsale di retraction contiene 931 celle. Dijkstra trova un percorso di 11 waypoint lungo 14.04 m, contro 11.31 m in linea retta tra partenza e arrivo (rapporto 1.24, valore stampato da `main_mobile` e salvato in `riepilogo.txt`): il percorso è più lungo perché passa sotto la parete e lontano dagli ostacoli. La distanza minima dagli ostacoli lungo il percorso pianificato è 0.570 m.

Il robot raggiunge la configurazione finale in 21.90 s con errore di posizione 0.0298 m (tolleranza 0.03 m) e di orientamento 0.0190 rad (tolleranza 0.02 rad). L'errore di tracking, cioè la distanza dal percorso pianificato, è al massimo 0.146 m e in media 0.031 m: il massimo si ha nelle curve, dove il robot taglia i waypoint entro la tolleranza di 0.3 m. La traiettoria eseguita resta a 0.549 m dagli ostacoli, ben oltre il raggio del robot (0.20 m).

Le ruote saturano a 15 rad/s: la velocità lineare massima è 0.750 m/s, mentre la velocità angolare massima del robot è 2.365 rad/s.

## Limiti

La retraction garantisce percorsi lontani dagli ostacoli ma non ottimi in lunghezza. La risoluzione della griglia limita i passaggi stretti: un corridoio più stretto del doppio dell'ingrandimento (0.7 m) risulta occupato. Il metodo è completo solo a meno della risoluzione scelta. Rispetto ai campi di potenziale non soffre di minimi locali, perché la ricerca con Dijkstra è globale.

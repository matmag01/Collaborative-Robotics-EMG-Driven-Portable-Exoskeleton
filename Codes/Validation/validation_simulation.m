% Parametri di input
massa = 80; % Massa del soggetto in kg
altezza = 1.80; % Altezza del soggetto in metri
g = 9.81; % Accelerazione gravitazionale (m/s^2)

% Parametri antropometrici (approssimazione media)
L_hand=(0.108*altezza);
lunghezza_avambraccio = 0.19; % Lunghezza approssimativa dell'avambraccio (in metri)
massa_avambraccio_e_mano = (0.016*massa) + (0.006*massa); % Massa approssimativa dell'avambraccio (in kg)
centro_di_massa = 0.682 * (lunghezza_avambraccio+L_hand); % Posizione del centro di massa (in metri)

% Angolo di riferimento (70 gradi)
angolo_desiderato = 30; 

% Calcolo del momento della forza peso a 70 gradi
momento_forza_peso = massa_avambraccio_e_mano * g * centro_di_massa * sind(angolo_desiderato); 

% Parametri del PID
Kp = 2; % Guadagno proporzionale
Ki = 0.01; % Guadagno integrale
Kd = 0.5; % Guadagno derivativo

% Inizializzazione del PID
errore_precedente = 0;
errore_integrale = 0;
momento_compensato = 0; % Momento compensato inizialmente a 0 Nm
momento_applicato = 0; % Momento applicato inizialmente a 0 Nm
posizione_corrente = angolo_desiderato; % La posizione corrente parte dalla posizione desiderata (70 gradi)

% Numero di passi della simulazione
num_steps = 100;

% Array per tracciare i momenti
momenti_tempo = zeros(1, num_steps);
momentum_error = zeros(1, num_steps);
momento_compensato_arr = zeros(1, num_steps);
posizione_corrente_arr = zeros(1, num_steps);
posizione_desiderata_arr = angolo_desiderato * ones(1, num_steps);

% Loop per la simulazione
for i = 1:num_steps
    % Generazione di un momento esterno casuale (ad esempio, 0Nm, 3Nm, 1Nm, 5Nm)
    momento_applicato = randi([0 5], 1, 1); % Moment randomici da 0 a 5 Nm
    
    % Calcolo dell'errore tra la posizione corrente e la posizione desiderata
    errore = angolo_desiderato - posizione_corrente;
    errore_integrale = errore_integrale + errore; % Somma dell'errore integrale
    errore_derivativo = errore - errore_precedente; % Differenza dell'errore
    
    % Calcolo del momento compensato tramite PID
    momento_compensato = Kp * errore + Ki * errore_integrale + Kd * errore_derivativo;
    
    % Simulazione della posizione corrente (aggiornamento dell'angolo)
    posizione_corrente = posizione_corrente + (momento_compensato - momento_forza_peso - momento_applicato) * 0.1; % Integrazione semplificata
    
    % Memorizzazione dei dati per analisi
    momenti_tempo(i) = momento_applicato;
    momentum_error(i) = errore;
    momento_compensato_arr(i) = momento_compensato;
    posizione_corrente_arr(i) = posizione_corrente;
    
    % Aggiornamento dell'errore precedente per il passo successivo
    errore_precedente = errore;
end

% Visualizzazione dei risultati
figure;

% Momenti applicati esterni
subplot(3,1,1);
plot(1:num_steps, momenti_tempo, 'r', 'LineWidth', 2);
title('Applied external torques');
xlabel('Sample');
ylabel('Applied torque (Nm)');
grid on;

% Errore tra posizione corrente e posizione desiderata
subplot(3,1,2);
plot(1:num_steps, momentum_error, 'b', 'LineWidth', 2);
title('Error between current position and desired position');
xlabel('Sample');
ylabel('Error [deg]');
grid on;

% Posizione corrente vs Posizione desiderata
subplot(3,1,3);
plot(1:num_steps, posizione_corrente_arr, 'g', 'LineWidth', 2);
hold on;
plot(1:num_steps, posizione_desiderata_arr, 'k--', 'LineWidth', 1);
title('Current position vs Desired position');
xlabel('Sample');
ylabel('Angle [deg]');
legend('Current position', 'Desired position');
grid on;

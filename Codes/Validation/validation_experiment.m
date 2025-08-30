% Apri il file
filename = 'test_PID_MicFINAL_100°.txt';
fileID = fopen(filename, 'r');

% Inizializza il vettore per salvare gli angoli
angles = [];

% Leggi il file riga per riga
while ~feof(fileID)
    line = fgetl(fileID); % Leggi una riga del file
    
    % Cerca la stringa "Angle:" e il valore numerico corrispondente
    match = regexp(line, 'Angle:\s*(-?\d+\.?\d*)', 'tokens');
    
    if ~isempty(match)
        % Estrai il valore numerico (dalla cella ritornata da regexp)
        angleValue = str2double(match{1}{1});
        % Salva il valore nel vettore degli angoli
        angles = [angles; angleValue];
    end
end

% Chiudi il file
fclose(fileID);

% Mostra i risultati
disp('Angoli estratti:');
disp(angles);

% (Opzionale) Plot degli angoli nel tempo
figure;
plot(angles, 'g-', 'LineWidth', 1.5);
xlabel('Sample');
ylabel('Angle [deg]');
title('Angle as a function of time');
grid on;


posizione_corrente = angles;  % Posizione corrente (supponendo sia la seconda colonna)

% Definizione della posizione desiderata
posizione_desiderata = 100; % Valore costante

% Calcolo dell'errore
errore = posizione_corrente - posizione_desiderata;

% Creazione del grafico
figure;

% Grafico dell'errore tra posizione corrente e desiderata
subplot(2,1,1);
plot(errore, 'b', 'LineWidth', 1.5);
title('Error between current position and desired position');
xlabel('Sample');
ylabel('Error [deg]');
grid on;

% Grafico Posizione Corrente vs Posizione Desiderata
subplot(2,1,2);
plot(posizione_corrente, 'g', 'LineWidth', 1.5);
hold on;
yline(posizione_desiderata, '--k', 'LineWidth', 1.5); % Linea orizzontale per la posizione desiderata
title('Current position vs Desired position');
xlabel('Sample');
ylabel('angle [deg]');
legend('Current position', 'Desired position');
grid on;
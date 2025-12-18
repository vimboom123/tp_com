%% Projet 3: Simulation complète Alamouti MISO (aligné avec main5)
% Basé sur main_5, utilisant les mêmes paramètres : 16-QAM + estimation par pilotes
% Conforme aux formules standard de décodage Alamouti (Slides 24-25)
clc; clear all; close all;

%% === Paramètres globaux (identiques à main5) ===
nb_data = 1000;             % Nombre de symboles de données par trame (pair, pour appariement Alamouti)
nb_pilot = 10;              % Nombre de symboles pilotes (identique à main5)
nb_bit_per_symb = 4;        % 16-QAM (identique à main5)
rolloff = 0.5;
symb_rate = 100e6;
sps = 2;
span = 16;
fs = symb_rate * sps;
nb_bit = nb_data * nb_bit_per_symb;

% Conception du filtre
g = raised_cosine(rolloff, span, sps, 'sqrt');

% Génération des symboles pilotes (identique à main5)
bit_pilot = randi([0 1], nb_pilot * nb_bit_per_symb, 1);
symb_pilot = mapping_QAM(bit_pilot, nb_bit_per_symb, length(bit_pilot));

%% ============================================================
%% PARTIE A: Visualisation d'une trame (pour présentation PPT)
%% ============================================================
fprintf('Exécution Partie A: Visualisation (SNR = 20dB, 16-QAM)...\n');

vis_snr_dB = 20; 
vis_snr_lin = 10^(vis_snr_dB/10);

% Génération d'une trame de données
bit_TX = randi([0 1], nb_bit, 1);
symb_TX_data = mapping_QAM(bit_TX, nb_bit_per_symb, nb_bit);

% --- Codage Alamouti (Slide 24) ---
% Appariement des symboles: (s1, s2), (s3, s4), ...
s1 = symb_TX_data(1:2:end);  % Symboles aux positions impaires
s2 = symb_TX_data(2:2:end);  % Symboles aux positions paires
nb_pairs = length(s1);

% Normalisation de puissance: chaque antenne émet 1/sqrt(2) fois
scale = 1/sqrt(2);

% Construction des séquences pour les deux antennes (2 intervalles par paire)
% Intervalle 2k-1: TX1 envoie s_k*scale, TX2 envoie s_{k+1}*scale
% Intervalle 2k:   TX1 envoie -conj(s_{k+1})*scale, TX2 envoie conj(s_k)*scale
symb_TX1 = zeros(nb_data, 1); 
symb_TX2 = zeros(nb_data, 1);
symb_TX1(1:2:end) = s1 * scale; 
symb_TX2(1:2:end) = s2 * scale;
symb_TX1(2:2:end) = -conj(s2) * scale;
symb_TX2(2:2:end) = conj(s1) * scale;

% --- Ajout des pilotes (Alamouti nécessite 2 intervalles orthogonaux pour estimer h1 et h2) ---
% Conception pilotes: intervalle 1 TX1 envoie pilot TX2 silence, intervalle 2 TX1 silence TX2 envoie pilot
pilot_ala = symb_pilot(1:min(nb_pilot, length(symb_pilot)));
nb_pilot_ala = length(pilot_ala);

% Utilisation de 2 intervalles pilotes orthogonaux
pilot_TX1 = [pilot_ala; zeros(nb_pilot_ala, 1)];  % TX1: pilot, 0
pilot_TX2 = [zeros(nb_pilot_ala, 1); pilot_ala];  % TX2: 0, pilot

% Trame complète: [Pilotes | Données]
frame_TX1 = [pilot_TX1; symb_TX1];
frame_TX2 = [pilot_TX2; symb_TX2];

% --- Mise en forme d'impulsion ---
sig_TX1 = convolution_TX(frame_TX1, g, sps);
sig_TX2 = convolution_TX(frame_TX2, g, sps);

% --- Figure 1: Spectre du signal transmis ---
figure('Name', 'Spectre TX');
f = linspace(-fs/2, fs/2, length(sig_TX1));
plot(f*1e-6, 10*log10(abs(fftshift(fft(sig_TX1)))));
grid on; title('Spectre Antenne TX 1 (Alamouti, 16-QAM)');
xlabel('Fréquence [MHz]'); ylabel('Amplitude [dB]');

% --- Canal de Rayleigh ---
H1_vis = (randn + 1i*randn)/sqrt(2);
H2_vis = (randn + 1i*randn)/sqrt(2);

% Signal reçu = h1*x1 + h2*x2
sig_RX_noiseless = H1_vis * sig_TX1 + H2_vis * sig_TX2;

% Ajout de bruit (définition SNR identique à main5)
rx_pwr = sum(abs(sig_TX1).^2 + abs(sig_TX2).^2)/length(sig_TX1)*sps;
noise_var = rx_pwr / vis_snr_lin;
noise = sqrt(noise_var/2)*(randn(size(sig_RX_noiseless)) + 1i*randn(size(sig_RX_noiseless)));
sig_RX = sig_RX_noiseless + noise;

% --- Filtrage de réception et échantillonnage ---
frame_len = 2*nb_pilot_ala + nb_data;
symb_RX_raw = convolution_RX(sig_RX, g, sps);
symb_RX_raw = symb_RX_raw(span+1:span+frame_len);

% --- Estimation du canal par pilotes (méthode LS similaire à main5) ---
% Estimation séparée de h1 et h2 à partir des pilotes orthogonaux
pilot_RX1 = symb_RX_raw(1:nb_pilot_ala);           % Intervalle où seul TX1 émet
pilot_RX2 = symb_RX_raw(nb_pilot_ala+1:2*nb_pilot_ala);  % Intervalle où seul TX2 émet

H1_est = sum(pilot_RX1 ./ pilot_ala) / nb_pilot_ala;  % Estimation LS de h1
H2_est = sum(pilot_RX2 ./ pilot_ala) / nb_pilot_ala;  % Estimation LS de h2

fprintf('Canal réel:  h1 = %.3f%+.3fi, h2 = %.3f%+.3fi\n', real(H1_vis), imag(H1_vis), real(H2_vis), imag(H2_vis));
fprintf('Canal estimé: h1 = %.3f%+.3fi, h2 = %.3f%+.3fi\n', real(H1_est), imag(H1_est), real(H2_est), imag(H2_est));

% Extraction de la partie données
symb_RX_data = symb_RX_raw(2*nb_pilot_ala+1:end);

% --- Figure 2: Constellations reçues ---
figure('Name', 'Constellations (16-QAM)');

subplot(1,3,1);
plot(real(symb_TX_data), imag(symb_TX_data), 'or', 'MarkerSize', 6);
grid on; title('1. Symboles TX (16-QAM)'); axis square; axis([-4 4 -4 4]);
xlabel('I'); ylabel('Q');

subplot(1,3,2);
plot(real(symb_RX_data), imag(symb_RX_data), 'x', 'MarkerSize', 4);
grid on; title('2. RX Brut (h_1 x_1 + h_2 x_2)'); axis square;
xlabel('I'); ylabel('Q');

% --- Décodage Alamouti (Formule standard Slide 25) ---
y1 = symb_RX_data(1:2:end);   % Réception intervalles impairs
y2 = symb_RX_data(2:2:end);   % Réception intervalles pairs

% Formule de décodage standard (avec canal estimé)
% z1 = (1/sqrt(2)) * (h1* y1 + h2 y2*)
% z2 = (1/sqrt(2)) * (h2* y1 - h1 y2*)
y2c = conj(y2);
z1 = (1/sqrt(2)) * (conj(H1_est).*y1 + H2_est.*y2c);
z2 = (1/sqrt(2)) * (conj(H2_est).*y1 - H1_est.*y2c);

% Égalisation: division par alpha = (|h1|^2 + |h2|^2)/2
alpha = (abs(H1_est)^2 + abs(H2_est)^2) / 2;
x1_est = z1 / alpha;
x2_est = z2 / alpha;

% Reconstruction de la séquence complète
symb_est = zeros(nb_data, 1);
symb_est(1:2:end) = x1_est;
symb_est(2:2:end) = x2_est;

subplot(1,3,3);
plot(real(symb_est), imag(symb_est), '.b', 'MarkerSize', 4);
hold on;
plot(real(symb_TX_data(1:50)), imag(symb_TX_data(1:50)), 'or', 'MarkerSize', 8, 'LineWidth', 1.5);
grid on; title('3. Après Décodage Alamouti'); axis square; axis([-4 4 -4 4]);
legend('Récupérés', 'Originaux (réf)', 'Location', 'northeast');
xlabel('I'); ylabel('Q');

%% ============================================================
%% PARTIE B: Analyse de Performance BER (Monte Carlo - comparaison avec main5)
%% ============================================================
fprintf('\nExécution Partie B: Simulation BER (16-QAM, estimation pilotes)...\n');
fprintf('Comparaison: SISO (baseline main5) vs Alamouti 2x1\n');

nb_frame_ber = 500;
Lsnr_dB = 0:2:20;
BER_Alamouti = zeros(1, length(Lsnr_dB));
BER_SISO = zeros(1, length(Lsnr_dB));

for k = 1:length(Lsnr_dB)
    snr_now = Lsnr_dB(k);
    snr_lin = 10^(snr_now/10);
    err_ala = 0; err_siso = 0;
    
    for i = 1:nb_frame_ber
        % Génération des données
        bits = randi([0 1], nb_bit, 1);
        syms_data = mapping_QAM(bits, nb_bit_per_symb, nb_bit);
        
        % Génération du canal (Rayleigh)
        h1 = (randn + 1i*randn)/sqrt(2);
        h2 = (randn + 1i*randn)/sqrt(2);
        h_siso = (randn + 1i*randn)/sqrt(2);
        
        %% === SISO (identique à main5) ===
        % Structure de trame: [Pilotes | Données]
        frame_siso = [symb_pilot; syms_data];
        sig_siso = convolution_TX(frame_siso, g, sps);
        
        % Passage dans le canal
        r_siso_nl = h_siso * sig_siso;
        p_siso = sum(abs(sig_siso).^2)/length(sig_siso)*sps;
        n_var_s = p_siso/snr_lin;
        noise_s = sqrt(n_var_s/2)*(randn(size(r_siso_nl))+1i*randn(size(r_siso_nl)));
        r_siso = r_siso_nl + noise_s;
        
        % Filtrage de réception
        symb_RX_siso = convolution_RX(r_siso, g, sps);
        symb_RX_siso = symb_RX_siso(span+1:span+nb_pilot+nb_data);
        
        % Estimation du canal par pilotes (identique à main5)
        H_est_siso = symb_RX_siso(1:nb_pilot) ./ symb_pilot;
        h_hat_siso = sum(H_est_siso) / nb_pilot;
        
        % Égalisation + Décision
        symb_RX_data_siso = symb_RX_siso(nb_pilot+1:end);
        symb_EQ_siso = symb_RX_data_siso / h_hat_siso;
        bits_siso = demapping_QAM(symbol_estimation_QAM(symb_EQ_siso, nb_bit_per_symb, nb_data), nb_bit_per_symb, nb_data);
        
        %% === Alamouti 2x1 ===
        % Codage Alamouti
        s1 = syms_data(1:2:end); s2 = syms_data(2:2:end);
        scale = 1/sqrt(2);
        tx1_data = zeros(nb_data,1); tx2_data = zeros(nb_data,1);
        tx1_data(1:2:end) = s1*scale; tx1_data(2:2:end) = -conj(s2)*scale;
        tx2_data(1:2:end) = s2*scale; tx2_data(2:2:end) = conj(s1)*scale;
        
        % Pilotes orthogonaux
        pilot_ala_len = nb_pilot;
        pilot_tx1 = [symb_pilot; zeros(pilot_ala_len, 1)];
        pilot_tx2 = [zeros(pilot_ala_len, 1); symb_pilot];
        
        frame_tx1 = [pilot_tx1; tx1_data];
        frame_tx2 = [pilot_tx2; tx2_data];
        
        sig_tx1 = convolution_TX(frame_tx1, g, sps);
        sig_tx2 = convolution_TX(frame_tx2, g, sps);
        
        % Passage dans le canal
        r_ala_nl = h1*sig_tx1 + h2*sig_tx2;
        p_ala = sum(abs(sig_tx1).^2 + abs(sig_tx2).^2)/length(sig_tx1)*sps;
        n_var_a = p_ala/snr_lin;
        noise_a = sqrt(n_var_a/2)*(randn(size(r_ala_nl))+1i*randn(size(r_ala_nl)));
        r_ala = r_ala_nl + noise_a;
        
        % Filtrage de réception
        frame_len_ala = 2*pilot_ala_len + nb_data;
        symb_RX_ala = convolution_RX(r_ala, g, sps);
        symb_RX_ala = symb_RX_ala(span+1:span+frame_len_ala);
        
        % Estimation du canal par pilotes
        pilot_rx1 = symb_RX_ala(1:pilot_ala_len);
        pilot_rx2 = symb_RX_ala(pilot_ala_len+1:2*pilot_ala_len);
        h1_hat = sum(pilot_rx1 ./ symb_pilot) / pilot_ala_len;
        h2_hat = sum(pilot_rx2 ./ symb_pilot) / pilot_ala_len;
        
        % Extraction des données
        symb_RX_data_ala = symb_RX_ala(2*pilot_ala_len+1:end);
        
        % Décodage Alamouti (Formule standard Slide 25)
        y1_r = symb_RX_data_ala(1:2:end);
        y2_r = symb_RX_data_ala(2:2:end);
        y2c = conj(y2_r);
        
        z1 = (1/sqrt(2)) * (conj(h1_hat).*y1_r + h2_hat.*y2c);
        z2 = (1/sqrt(2)) * (conj(h2_hat).*y1_r - h1_hat.*y2c);
        
        alpha = (abs(h1_hat)^2 + abs(h2_hat)^2) / 2;
        x1_eq = z1 / alpha;
        x2_eq = z2 / alpha;
        
        rec_ala = zeros(nb_data, 1);
        rec_ala(1:2:end) = x1_eq;
        rec_ala(2:2:end) = x2_eq;
        
        bits_ala = demapping_QAM(symbol_estimation_QAM(rec_ala, nb_bit_per_symb, nb_data), nb_bit_per_symb, nb_data);
        
        %% Comptage des erreurs
        [~,e1] = biterr(bits, bits_ala);
        [~,e2] = biterr(bits, bits_siso);
        err_ala = err_ala + e1;
        err_siso = err_siso + e2;
    end
    
    BER_Alamouti(k) = err_ala / nb_frame_ber;
    BER_SISO(k) = err_siso / nb_frame_ber;
    fprintf('SNR %2d dB: BER_SISO = %.2e, BER_Alamouti = %.2e\n', snr_now, BER_SISO(k), BER_Alamouti(k));
end

%% --- Figure 3: Courbes BER ---
figure('Name', 'Comparaison BER (16-QAM, Estimation Pilotes)');
semilogy(Lsnr_dB, BER_SISO, '-o', 'LineWidth', 2, 'MarkerSize', 8, 'DisplayName', 'SISO (baseline main5)');
hold on;
semilogy(Lsnr_dB, BER_Alamouti, '-^', 'LineWidth', 2, 'MarkerSize', 8, 'DisplayName', 'Alamouti 2x1 MISO');

% Courbes théoriques (fonctions MATLAB intégrées)
EbNo_dB = Lsnr_dB - 10*log10(nb_bit_per_symb);
M = 2^nb_bit_per_symb;  % 16-QAM

% BER théorique Rayleigh
ber_theory_siso = berfading(EbNo_dB, 'qam', M, 1);
ber_theory_ala = berfading(EbNo_dB, 'qam', M, 2);
semilogy(Lsnr_dB, ber_theory_siso, '--k', 'LineWidth', 1.5, 'DisplayName', 'Théorie SISO (Rayleigh)');
semilogy(Lsnr_dB, ber_theory_ala, '--r', 'LineWidth', 1.5, 'DisplayName', 'Théorie Alamouti (div=2)');

grid on;
xlabel('SNR (dB)');
ylabel('Taux d''Erreur Binaire (BER)');
title('Projet 3: Alamouti STBC vs SISO (16-QAM, Canal Rayleigh)');
legend('Location', 'southwest');
ylim([1e-5 1]);

fprintf('\n=== Simulation terminée ===\n');
fprintf('Modulation: 16-QAM (identique à main5)\n');
fprintf('Estimation canal: Pilotes LS (identique à main5)\n');
fprintf('Décodage Alamouti: Formule standard Slides 24-25\n');

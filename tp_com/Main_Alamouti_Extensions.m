%% Project 3 (Alamouti) - Extensions file
% Objective: Without modifying the original Project 3 script (Main_Alamouti_Full_Project.m),
%            implement and evaluate 3 extension directions:
%   1) Additional channel model: Rice (Rician) channel, sweep K factor
%   2) More realistic channel characteristics: Tx antenna spatial correlation (h1 and h2 correlated in 2x1 MISO, parameter rho)
%   3) More realistic time-varying channel: Doppler-induced fading (normalized Doppler: fdTs = f_D * T_symb)
%
% Output: Each extension direction generates one BER vs SNR curve plot (multiple curves for comparison)
%
% Dependencies (same as original project):
% - mapping_QAM.m / demapping_QAM.m / symbol_estimation_QAM.m
% - raised_cosine.m / convolution_TX.m / convolution_RX.m
% - Communications Toolbox: biterr (if your original script runs, this one will too)
%
% Notes:
% - Doppler time-varying channel uses "simplified Jakes autocorrelation + AR(1) approximation" to generate Rayleigh/Rice fading sequences
% - This script prioritizes readability and presentation; for smoother BER curves, increase nb_frame_ber
clc; close all;
% Allow pre-setting these switches in workspace before running the script (they won't be cleared)
clearvars -except fast_mode run_rice_sweep run_corr_sweep run_doppler_sweep doppler_tracking doppler_data_seg_len;

%% ========== Global Parameters (aligned with Main_Alamouti_Full_Project.m) ==========
% Run switches (to save time, you can run only one sweep)
if ~exist('run_rice_sweep', 'var'); run_rice_sweep = true; end
if ~exist('run_corr_sweep', 'var'); run_corr_sweep = true; end
if ~exist('run_doppler_sweep', 'var'); run_doppler_sweep = true; end

% Fast mode: default false (more accurate); set to true for faster debugging
if ~exist('fast_mode', 'var'); fast_mode = false; end

% Doppler optimization: perform "periodic pilot tracking" for time-varying channels to avoid BER collapsing to 0.5
% - doppler_tracking=true: insert orthogonal pilots every doppler_data_seg_len data symbols and update (h1,h2) estimate
% - doppler_tracking=false: use only frame-head pilot estimation (high-speed Doppler will easily cause BER floor ~ 0.5)
if ~exist('doppler_tracking', 'var'); doppler_tracking = true; end
if ~exist('doppler_data_seg_len', 'var'); doppler_data_seg_len = 100; end   % Must be even; recommended 50~200

nb_data = 1000;             % Number of data symbols per frame (must be even for Alamouti pair encoding)
nb_pilot = 10;              % Number of pilot symbols (each antenna occupies nb_pilot time slots, total pilot overhead = 2*nb_pilot)
nb_bit_per_symb = 4;        % 16-QAM
rolloff = 0.5;
symb_rate = 100e6;
sps = 2;
span = 16;
fs = symb_rate * sps;

nb_bit = nb_data * nb_bit_per_symb;
g = raised_cosine(rolloff, span, sps, 'sqrt');

% BER simulation settings
if fast_mode
    Lsnr_dB = 0:4:20;
    nb_frame_ber = 120;
else
    Lsnr_dB = 0:2:20;
    nb_frame_ber = 500;
end

% For reproducibility (you can comment this out)
rng(1);

% Generate pilot once (all sweeps reuse the same pilot for fair comparison)
bit_pilot = randi([0 1], nb_pilot * nb_bit_per_symb, 1);
symb_pilot = mapping_QAM(bit_pilot, nb_bit_per_symb, length(bit_pilot));

fprintf('=== Main_Alamouti_Extensions.m ===\n');
fprintf('16-QAM, nb_data=%d, nb_pilot=%d (orthogonal pilots => 2*nb_pilot), sps=%d, span=%d\n', nb_data, nb_pilot, sps, span);
fprintf('SNR sweep: %d points, Monte-Carlo frames per point: %d\n\n', length(Lsnr_dB), nb_frame_ber);

%% ============================================================
%% 1) Rice(K) sweep: Additional channel model (Rician)
%% ============================================================
if fast_mode
    K_dB_list = [0 10];     % Fast mode: fewer points
else
    K_dB_list = [0 5 10];   % Full mode
end
rho = 0;               % No spatial correlation
fdTs = 0;              % Block fading (constant channel per frame)

BER_Rice = zeros(length(K_dB_list), length(Lsnr_dB));

if run_rice_sweep
    fprintf('[1/3] Rice(K) sweep...\n');
    for iK = 1:length(K_dB_list)
        K = 10^(K_dB_list(iK)/10);
        chanOpt = struct('type', 'Rice', 'K', K, 'rho', rho, 'fdTs', fdTs);
        BER_Rice(iK, :) = sim_ber_alamouti(Lsnr_dB, nb_frame_ber, ...
            nb_data, nb_bit, nb_bit_per_symb, nb_pilot, symb_pilot, ...
            g, sps, span, chanOpt);
        fprintf('  K=%g (%.1f dB) done\n', K, K_dB_list(iK));
    end
    
    figure('Name', 'BER vs SNR - Rice K sweep (Alamouti 2x1)');
    hold on; grid on;
    for iK = 1:length(K_dB_list)
        semilogy(Lsnr_dB, BER_Rice(iK, :), '-o', 'LineWidth', 2, 'MarkerSize', 7, ...
            'DisplayName', sprintf('Rice, K=%.1f dB', K_dB_list(iK)));
    end
    xlabel('SNR (dB)');
    ylabel('BER');
    title('Alamouti 2x1 - Rice(K) sweep (channel block fading)');
    legend('Location', 'southwest');
    ylim([1e-5 1]);
end

%% ============================================================
%% 2) Correlation sweep: Tx spatial correlation (h1 and h2 correlated)
%% ============================================================
if fast_mode
    rho_list = [0 0.9];
else
    rho_list = [0 0.5 0.9];
end
K = 0;                 % Rayleigh
fdTs = 0;              % Block fading

BER_Rho = zeros(length(rho_list), length(Lsnr_dB));

if run_corr_sweep
    fprintf('\n[2/3] Spatial correlation sweep...\n');
    for ir = 1:length(rho_list)
        rho = rho_list(ir);
        chanOpt = struct('type', 'Rayleigh', 'K', K, 'rho', rho, 'fdTs', fdTs);
        BER_Rho(ir, :) = sim_ber_alamouti(Lsnr_dB, nb_frame_ber, ...
            nb_data, nb_bit, nb_bit_per_symb, nb_pilot, symb_pilot, ...
            g, sps, span, chanOpt);
        fprintf('  rho=%0.2f done\n', rho);
    end
    
    figure('Name', 'BER vs SNR - Spatial correlation sweep (Alamouti 2x1)');
    hold on; grid on;
    for ir = 1:length(rho_list)
        semilogy(Lsnr_dB, BER_Rho(ir, :), '-^', 'LineWidth', 2, 'MarkerSize', 7, ...
            'DisplayName', sprintf('Rayleigh, \\rho=%0.2f', rho_list(ir)));
    end
    xlabel('SNR (dB)');
    ylabel('BER');
    title('Alamouti 2x1 - Tx spatial correlation sweep (channel block fading)');
    legend('Location', 'southwest');
    ylim([1e-5 1]);
end

%% ============================================================
%% 3) Doppler(fdTs) sweep: Time-varying channel (normalized Doppler = f_D * T_symb)
%% ============================================================
% The larger fdTs, the faster the channel varies; Alamouti assumes "channel constant over 2 time slots"
% Therefore, as fdTs increases, performance typically degrades significantly (especially when channel changes noticeably within 2 slots)
if fast_mode
    % Previously using 0.2/0.5 caused "almost complete mismatch" with BER ~ 0.5, now using more gradual values
    fdTs_list = [0 0.005 0.01 0.02];
else
    fdTs_list = [0 0.002 0.005 0.01 0.02];
end
rho = 0;                       % No spatial correlation
K = 0;                         % Rayleigh

BER_Doppler = zeros(length(fdTs_list), length(Lsnr_dB));

if run_doppler_sweep
    fprintf('\n[3/3] Doppler (time-varying fading) sweep...\n');
    if doppler_tracking
        fprintf('  Doppler tracking: ON (pilot every %d data symbols)\n', doppler_data_seg_len);
    else
        fprintf('  Doppler tracking: OFF (single pilot at frame head)\n');
    end
    for id = 1:length(fdTs_list)
        fdTs = fdTs_list(id);
        chanOpt = struct('type', 'Rayleigh', 'K', K, 'rho', rho, 'fdTs', fdTs);
        if doppler_tracking
            chanOpt.pilot_mode = 'periodic';
            chanOpt.data_seg_len = doppler_data_seg_len;
        else
            chanOpt.pilot_mode = 'single';
        end
        BER_Doppler(id, :) = sim_ber_alamouti(Lsnr_dB, nb_frame_ber, ...
            nb_data, nb_bit, nb_bit_per_symb, nb_pilot, symb_pilot, ...
            g, sps, span, chanOpt);
        fprintf('  fdTs=%0.3f done (a=J0(2*pi*fdTs)=%0.3f)\n', fdTs, besselj(0, 2*pi*fdTs));
    end
    
    figure('Name', 'BER vs SNR - Doppler sweep (Alamouti 2x1)');
    hold on; grid on;
    for id = 1:length(fdTs_list)
        semilogy(Lsnr_dB, BER_Doppler(id, :), '-s', 'LineWidth', 2, 'MarkerSize', 7, ...
            'DisplayName', sprintf('Rayleigh, fdTs=%0.3f', fdTs_list(id)));
    end
    xlabel('SNR (dB)');
    ylabel('BER');
    title('Alamouti 2x1 - Doppler time-varying fading sweep');
    legend('Location', 'southwest');
    ylim([1e-5 1]);
end

fprintf('\n=== Done ===\n');
fprintf('You can now use these 3 figures directly in the "Extension/Discussion" section of your presentation.\n');

%% =====================================================================
%% Local functions
%% =====================================================================
function BER = sim_ber_alamouti(Lsnr_dB, nb_frame_ber, ...
    nb_data, nb_bit, nb_bit_per_symb, nb_pilot, symb_pilot, ...
    g, sps, span, chanOpt)

BER = zeros(1, length(Lsnr_dB));

% Pilot mode: 'single' (frame head only) or 'periodic' (segmented tracking)
if isfield(chanOpt, 'pilot_mode')
    pilot_mode = chanOpt.pilot_mode;
else
    pilot_mode = 'single';
end

if strcmpi(pilot_mode, 'periodic')
    if ~isfield(chanOpt, 'data_seg_len')
        error('pilot_mode=periodic requires chanOpt.data_seg_len to be provided');
    end
    data_seg_len = chanOpt.data_seg_len;
    if mod(data_seg_len, 2) ~= 0
        error('data_seg_len must be even (Alamouti decodes in pairs).');
    end
    if mod(nb_data, data_seg_len) ~= 0
        error('nb_data must be divisible by data_seg_len for segmented processing.');
    end
    nSeg = nb_data / data_seg_len;
end

for k = 1:length(Lsnr_dB)
    snr_lin = 10^(Lsnr_dB(k)/10);
    ber_acc = 0;

    for iFrm = 1:nb_frame_ber
        %% ===== TX: data generation + QAM mapping =====
        bits = randi([0 1], nb_bit, 1);
        syms_data = mapping_QAM(bits, nb_bit_per_symb, nb_bit);

        %% ===== TX: Alamouti encoding (2x1) =====
        s1 = syms_data(1:2:end);
        s2 = syms_data(2:2:end);
        scale = 1/sqrt(2);

        tx1_data = zeros(nb_data, 1);
        tx2_data = zeros(nb_data, 1);
        tx1_data(1:2:end) = s1 * scale;
        tx2_data(1:2:end) = s2 * scale;
        tx1_data(2:2:end) = -conj(s2) * scale;
        tx2_data(2:2:end) = conj(s1) * scale;

        %% ===== Pilots (orthogonal in time) =====
        pilot_len = nb_pilot;
        pilot_tx1 = [symb_pilot; zeros(pilot_len, 1)];
        pilot_tx2 = [zeros(pilot_len, 1); symb_pilot];

        if strcmpi(pilot_mode, 'single')
            frame_tx1 = [pilot_tx1; tx1_data];
            frame_tx2 = [pilot_tx2; tx2_data];
            frame_len_symb = 2*pilot_len + nb_data;
        else
            % Periodic: [pilots | data_seg] repeated
            frame_tx1 = [];
            frame_tx2 = [];
            for iseg = 1:nSeg
                idx1 = (iseg-1)*data_seg_len + 1;
                idx2 = iseg*data_seg_len;
                frame_tx1 = [frame_tx1; pilot_tx1; tx1_data(idx1:idx2)]; %#ok<AGROW>
                frame_tx2 = [frame_tx2; pilot_tx2; tx2_data(idx1:idx2)]; %#ok<AGROW>
            end
            frame_len_symb = nSeg * (2*pilot_len + data_seg_len);
        end

        %% ===== Pulse shaping =====
        sig_tx1 = convolution_TX(frame_tx1, g, sps);
        sig_tx2 = convolution_TX(frame_tx2, g, sps);

        %% ===== Channel (2x1 MISO) =====
        [h1_samp, h2_samp] = gen_miso_2x1_channel_samples(length(sig_tx1), sps, span, chanOpt);
        r_nl = h1_samp .* sig_tx1 + h2_samp .* sig_tx2;

        %% ===== AWGN =====
        p_ala = sum(abs(sig_tx1).^2 + abs(sig_tx2).^2) / length(sig_tx1) * sps;
        n_var = p_ala / snr_lin;
        noise = sqrt(n_var/2) * (randn(size(r_nl)) + 1i*randn(size(r_nl)));
        r = r_nl + noise;

        %% ===== RX matched filter + sampling =====
        symb_rx = convolution_RX(r, g, sps);
        symb_rx = symb_rx(span+1 : span+frame_len_symb);

        %% ===== Channel estimation + Alamouti decoding =====
        rec = zeros(nb_data, 1);
        if strcmpi(pilot_mode, 'single')
            % --- Pilot at frame head ---
            pilot_rx1 = symb_rx(1:pilot_len);
            pilot_rx2 = symb_rx(pilot_len+1 : 2*pilot_len);

            h1_hat = sum(pilot_rx1 ./ symb_pilot) / pilot_len;
            h2_hat = sum(pilot_rx2 ./ symb_pilot) / pilot_len;

            symb_rx_data = symb_rx(2*pilot_len+1 : end);
            y1 = symb_rx_data(1:2:end);
            y2 = symb_rx_data(2:2:end);
            y2c = conj(y2);

            z1 = (1/sqrt(2)) * (conj(h1_hat).*y1 + h2_hat.*y2c);
            z2 = (1/sqrt(2)) * (conj(h2_hat).*y1 - h1_hat.*y2c);

            alpha = (abs(h1_hat)^2 + abs(h2_hat)^2) / 2;
            x1_eq = z1 / alpha;
            x2_eq = z2 / alpha;

            rec(1:2:end) = x1_eq;
            rec(2:2:end) = x2_eq;
        else
            % --- Periodic pilots: estimate per segment, then decode segment data ---
            blkLen = 2*pilot_len + data_seg_len;
            for iseg = 1:nSeg
                blkStart = (iseg-1)*blkLen + 1;

                pilot_rx1 = symb_rx(blkStart : blkStart+pilot_len-1);
                pilot_rx2 = symb_rx(blkStart+pilot_len : blkStart+2*pilot_len-1);

                h1_hat = sum(pilot_rx1 ./ symb_pilot) / pilot_len;
                h2_hat = sum(pilot_rx2 ./ symb_pilot) / pilot_len;

                dataStart = blkStart + 2*pilot_len;
                dataBlk = symb_rx(dataStart : dataStart+data_seg_len-1);

                y1 = dataBlk(1:2:end);
                y2 = dataBlk(2:2:end);
                y2c = conj(y2);

                z1 = (1/sqrt(2)) * (conj(h1_hat).*y1 + h2_hat.*y2c);
                z2 = (1/sqrt(2)) * (conj(h2_hat).*y1 - h1_hat.*y2c);

                alpha = (abs(h1_hat)^2 + abs(h2_hat)^2) / 2;
                x1_eq = z1 / alpha;
                x2_eq = z2 / alpha;

                recIdx1 = (iseg-1)*data_seg_len + 1;
                recIdx2 = iseg*data_seg_len;
                rec(recIdx1:2:recIdx2) = x1_eq;
                rec(recIdx1+1:2:recIdx2) = x2_eq;
            end
        end

        %% ===== Hard decision + demap =====
        symb_hat = symbol_estimation_QAM(rec, nb_bit_per_symb, nb_data);
        bits_hat = demapping_QAM(symb_hat, nb_bit_per_symb, nb_data);

        %% ===== BER accumulate =====
        [~, ber_frame] = biterr(bits, bits_hat);
        ber_acc = ber_acc + ber_frame;
    end

    BER(k) = ber_acc / nb_frame_ber;
end
end

function [h1_samp, h2_samp] = gen_miso_2x1_channel_samples(nb_samp, sps, span, chanOpt)
% Output h1_samp/h2_samp: column vectors of length nb_samp (channel coefficient for each sample)
%
% Generation approach:
% - First generate channel sequence H_symb at "symbol rate" (length ~ nb_samp/sps)
% - Then repeat each symbol's channel coefficient sps times to expand to sample points

rho = chanOpt.rho;
if abs(rho) >= 1
    error('rho must satisfy |rho|<1 to ensure covariance matrix is positive definite.');
end

% Number of "symbol intervals" needed: use ceil(nb_samp/sps) + span for padding to avoid boundary issues
nb_symb = ceil(nb_samp / sps) + span;

% Doppler: use normalized fdTs = f_D * T_symb, Jakes autocorrelation R(1)=J0(2*pi*fdTs)
fdTs = chanOpt.fdTs;
if fdTs < 0
    error('fdTs must be >= 0');
end

if fdTs == 0
    a = 1;
else
    a = besselj(0, 2*pi*fdTs);
    % Numerical safety: avoid |a|>1 causing sqrt(1-a^2) to be complex
    a = max(min(a, 1), -1);
end

% Antenna correlation covariance matrix (2x2)
R = [1 rho; rho 1];
L = chol(R, 'lower');

% Generate Rayleigh NLOS: 2 x nb_symb
U = (randn(2, nb_symb) + 1i*randn(2, nb_symb)) / sqrt(2);
W = L * U;

H_nlos = zeros(2, nb_symb);
H_nlos(:, 1) = W(:, 1);
if a == 1
    H_nlos(:, 2:end) = repmat(H_nlos(:, 1), 1, nb_symb-1);
else
    b = sqrt(1 - a^2);
    for n = 2:nb_symb
        H_nlos(:, n) = a * H_nlos(:, n-1) + b * W(:, n);
    end
end

% Rayleigh / Rice
switch chanOpt.type
    case 'Rayleigh'
        H = H_nlos;
    case 'Rice'
        K = chanOpt.K;
        if K < 0
            error('Rice K must be >= 0');
        end
        % LOS: each antenna has a fixed phase (randomized once per frame)
        phi = 2*pi*rand(2, 1);
        H_los = exp(1i*phi) * ones(1, nb_symb);
        H = sqrt(K/(1+K)) * H_los + sqrt(1/(1+K)) * H_nlos;
    otherwise
        error('Unknown channel type: %s (supported: Rayleigh / Rice)', chanOpt.type);
end

% Expand to sample points: repeat each symbol sps times
H_samp = kron(H, ones(1, sps));    % 2 x (nb_symb*sps)
H_samp = H_samp(:, 1:nb_samp);

h1_samp = H_samp(1, :).';
h2_samp = H_samp(2, :).';
end

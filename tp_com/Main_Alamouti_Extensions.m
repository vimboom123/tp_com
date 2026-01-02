%% Project 3 (Alamouti) - Extensions file
% 目标：在不改动原始 Projet 3 脚本(Main_Alamouti_Full_Project.m)的情况下，
%      额外实现/评估 3 个拓展方向：
%   1) 额外信道模型：Rice (Rician) 信道，扫 K 因子
%   2) 更真实的信道特性：发射天线间空间相关性（2x1 MISO 的 h1 与 h2 相关，参数 rho）
%   3) 更真实的时变信道：Doppler 导致的时变衰落（用归一化 Doppler：fdTs = f_D * T_symb）
%
% 输出：每个拓展方向各自生成一张 BER vs SNR 曲线图（多条曲线对比）
%
% 依赖（与原项目一致）：
% - mapping_QAM.m / demapping_QAM.m / symbol_estimation_QAM.m
% - raised_cosine.m / convolution_TX.m / convolution_RX.m
% - Communications Toolbox: biterr (若你原脚本能跑，这里也能跑)
%
% 备注：
% - Doppler 时变信道这里使用“简化 Jakes 自相关 + AR(1) 近似”生成 Rayleigh/Rice 衰落序列
% - 该脚本默认注重可读性与可展示性；想更平滑的 BER 曲线可增大 nb_frame_ber
clc; close all;
% 允许你在运行脚本前在 workspace 里预先设置下面这些开关（不会被清掉）
clearvars -except fast_mode run_rice_sweep run_corr_sweep run_doppler_sweep;

%% ========== 全局参数（与 Main_Alamouti_Full_Project.m 对齐） ==========
% 运行开关（为了节省时间，你也可以只跑其中一个 sweep）
if ~exist('run_rice_sweep', 'var'); run_rice_sweep = true; end
if ~exist('run_corr_sweep', 'var'); run_corr_sweep = true; end
if ~exist('run_doppler_sweep', 'var'); run_doppler_sweep = true; end

% 快速模式：默认 false（更精确）；调试时可设为 true 加快速度
if ~exist('fast_mode', 'var'); fast_mode = false; end

% Doppler 优化：对时变信道做"分段导频追踪"，避免 BER 直接崩到 0.5
% - doppler_tracking=true: 每隔 doppler_data_seg_len 个数据符号插入一次正交导频并更新 (h1,h2) 估计
% - doppler_tracking=false: 只用帧头一次导频估计（高速 Doppler 下会很容易出现 BER floor≈0.5）
if ~exist('doppler_tracking', 'var'); doppler_tracking = false; end
if ~exist('doppler_data_seg_len', 'var'); doppler_data_seg_len = 100; end   % 必须为偶数；建议 50~200

nb_data = 1000;             % 每帧数据符号数（必须为偶数，Alamouti 成对编码）
nb_pilot = 10;              % pilot 符号数（每根天线各占 nb_pilot 个时隙，总 pilot 开销 2*nb_pilot）
nb_bit_per_symb = 4;        % 16-QAM
rolloff = 0.5;
symb_rate = 100e6;
sps = 2;
span = 16;
fs = symb_rate * sps;

nb_bit = nb_data * nb_bit_per_symb;
g = raised_cosine(rolloff, span, sps, 'sqrt');

% BER 仿真设置
if fast_mode
    Lsnr_dB = 0:4:20;
    nb_frame_ber = 120;
else
    Lsnr_dB = 0:2:20;
    nb_frame_ber = 500;
end

% 为了可复现（你也可以注释掉）
rng(1);

% 生成一次 pilot（所有 sweep 复用同一组 pilot，方便公平对比）
bit_pilot = randi([0 1], nb_pilot * nb_bit_per_symb, 1);
symb_pilot = mapping_QAM(bit_pilot, nb_bit_per_symb, length(bit_pilot));

fprintf('=== Main_Alamouti_Extensions.m ===\n');
fprintf('16-QAM, nb_data=%d, nb_pilot=%d (orthogonal pilots => 2*nb_pilot), sps=%d, span=%d\n', nb_data, nb_pilot, sps, span);
fprintf('SNR sweep: %d points, Monte-Carlo frames per point: %d\n\n', length(Lsnr_dB), nb_frame_ber);

%% ============================================================
%% 1) Rice(K) sweep：额外信道模型（Rician）
%% ============================================================
if fast_mode
    K_dB_list = [0 10];     % 快速模式：少跑一点点
else
    K_dB_list = [0 5 10];   % 完整模式
end
rho = 0;               % 无空间相关
fdTs = 0;              % block fading（每帧常数信道）

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
%% 2) 相关性 sweep：Tx 空间相关性（h1 与 h2 相关）
%% ============================================================
if fast_mode
    rho_list = [0 0.9];
else
    rho_list = [0 0.5 0.9];
end
K = 0;                 % Rayleigh
fdTs = 0;              % block fading

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
%% 3) Doppler(fdTs) sweep：时变信道（归一化 Doppler = f_D * T_symb）
%% ============================================================
% fdTs 越大，信道随时间变化越快；Alamouti 假设“两时隙内信道不变”
% 因此 fdTs 增大时，一般会看到性能明显劣化（尤其在两时隙内变化明显时）
if fast_mode
    % 之前用 0.2/0.5 会导致“几乎完全失配”而 BER≈0.5，这里换成更渐进的数值
    fdTs_list = [0 0.005 0.01 0.02];
else
    fdTs_list = [0 0.002 0.005 0.01 0.02];
end
rho = 0;                       % 无空间相关
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
fprintf('你现在可以把这 3 张图直接用在答辩 PPT 的“扩展/讨论”部分。\n');

%% =====================================================================
%% Local functions
%% =====================================================================
function BER = sim_ber_alamouti(Lsnr_dB, nb_frame_ber, ...
    nb_data, nb_bit, nb_bit_per_symb, nb_pilot, symb_pilot, ...
    g, sps, span, chanOpt)

BER = zeros(1, length(Lsnr_dB));

% pilot mode: 'single' (帧头一次) or 'periodic' (分段追踪)
if isfield(chanOpt, 'pilot_mode')
    pilot_mode = chanOpt.pilot_mode;
else
    pilot_mode = 'single';
end

if strcmpi(pilot_mode, 'periodic')
    if ~isfield(chanOpt, 'data_seg_len')
        error('pilot_mode=periodic 时必须提供 chanOpt.data_seg_len');
    end
    data_seg_len = chanOpt.data_seg_len;
    if mod(data_seg_len, 2) ~= 0
        error('data_seg_len 必须为偶数（Alamouti 成对解码）。');
    end
    if mod(nb_data, data_seg_len) ~= 0
        error('nb_data 必须能被 data_seg_len 整除，方便分段处理。');
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

        %% ===== pilots (orthogonal in time) =====
        pilot_len = nb_pilot;
        pilot_tx1 = [symb_pilot; zeros(pilot_len, 1)];
        pilot_tx2 = [zeros(pilot_len, 1); symb_pilot];

        if strcmpi(pilot_mode, 'single')
            frame_tx1 = [pilot_tx1; tx1_data];
            frame_tx2 = [pilot_tx2; tx2_data];
            frame_len_symb = 2*pilot_len + nb_data;
        else
            % periodic: [pilots | data_seg] repeated
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

        %% ===== pulse shaping =====
        sig_tx1 = convolution_TX(frame_tx1, g, sps);
        sig_tx2 = convolution_TX(frame_tx2, g, sps);

        %% ===== channel (2x1 MISO) =====
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

        %% ===== channel estimation + Alamouti decoding =====
        rec = zeros(nb_data, 1);
        if strcmpi(pilot_mode, 'single')
            % --- pilot at frame head ---
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
            % --- periodic pilots: estimate per segment, then decode segment data ---
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

        %% ===== hard decision + demap =====
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
% 输出 h1_samp/h2_samp：长度 nb_samp 的列向量（对每个采样点的信道系数）
%
% 生成思路：
% - 先按“符号速率”生成信道序列 H_symb（长度约为 nb_samp/sps）
% - 再把每个符号的信道系数重复 sps 次扩展到采样点

rho = chanOpt.rho;
if abs(rho) >= 1
    error('rho 必须满足 |rho|<1 才能保证协方差矩阵正定。');
end

% 需要的“符号间隔”数量：用 ceil(nb_samp/sps) 再 +span 做 padding，避免边界不足
nb_symb = ceil(nb_samp / sps) + span;

% Doppler：使用归一化 fdTs = f_D * T_symb，Jakes 自相关 R(1)=J0(2*pi*fdTs)
fdTs = chanOpt.fdTs;
if fdTs < 0
    error('fdTs 必须 >= 0');
end

if fdTs == 0
    a = 1;
else
    a = besselj(0, 2*pi*fdTs);
    % 数值安全：避免 |a|>1 导致 sqrt(1-a^2) 复数
    a = max(min(a, 1), -1);
end

% 天线相关性协方差矩阵（2x2）
R = [1 rho; rho 1];
L = chol(R, 'lower');

% 生成 Rayleigh NLOS：2 x nb_symb
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
            error('Rice K 必须 >= 0');
        end
        % LOS：每根天线一个固定相位（每帧随机一次即可）
        phi = 2*pi*rand(2, 1);
        H_los = exp(1i*phi) * ones(1, nb_symb);
        H = sqrt(K/(1+K)) * H_los + sqrt(1/(1+K)) * H_nlos;
    otherwise
        error('未知 channel type: %s (支持 Rayleigh / Rice)', chanOpt.type);
end

% 扩展到采样点：每个符号重复 sps 次
H_samp = kron(H, ones(1, sps));    % 2 x (nb_symb*sps)
H_samp = H_samp(:, 1:nb_samp);

h1_samp = H_samp(1, :).';
h2_samp = H_samp(2, :).';
end



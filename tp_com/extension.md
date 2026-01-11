# Main_Alamouti_Extensions.m: Line-by-Line Code Explanation

This document provides a **comprehensive, line-by-line explanation** of every code statement in `Main_Alamouti_Extensions.m`, explaining what each line does and why it is necessary.

---

## Table of Contents

1. [File Header and Initialization](#file-header-and-initialization)
2. [Global Parameters](#global-parameters)
3. [Extension 1: Rice (Rician) Channel Sweep](#extension-1-rice-rician-channel-sweep)
4. [Extension 2: Spatial Correlation Sweep](#extension-2-spatial-correlation-sweep)
5. [Extension 3: Doppler Time-Varying Fading Sweep](#extension-3-doppler-time-varying-fading-sweep)
6. [Local Function: sim_ber_alamouti()](#local-function-sim_ber_alamouti)
7. [Local Function: gen_miso_2x1_channel_samples()](#local-function-gen_miso_2x1_channel_samples)

---

## File Header and Initialization

### Lines 1-17: Header Comments

```matlab
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
```

**Explanation:**
- **Line 1**: `%%` creates a section header in MATLAB, which appears in the editor's section navigation.
- **Lines 2-6**: Document the script's objective: extend the base Alamouti project with three new channel models without modifying the original code.
- **Lines 8-13**: List required helper functions and toolboxes. These must be in the MATLAB path.
- **Lines 15-17**: Important implementation notes: the Doppler channel uses an AR(1) model approximation of the Jakes autocorrelation, and BER accuracy can be improved by increasing `nb_frame_ber`.

### Line 18: Clear Command Window and Close Figures

```matlab
clc; close all;
```

**Explanation:**
- `clc`: Clears the command window, removing previous output for a clean start.
- `close all`: Closes all open figure windows to prevent clutter from previous runs.

### Line 20: Selective Variable Clearing

```matlab
clearvars -except fast_mode run_rice_sweep run_corr_sweep run_doppler_sweep doppler_tracking doppler_data_seg_len;
```

**Explanation:**
- `clearvars`: Removes all variables from the workspace.
- `-except`: Exception list. Variables listed after `-except` are **preserved**.
- **Why this matters**: If you set these flags in the workspace before running the script (e.g., `fast_mode = true;`), they will persist. This allows you to pre-configure the script without editing the code.

---

## Global Parameters

### Lines 22-26: Run Switches Initialization

```matlab
%% ========== Global Parameters (aligned with Main_Alamouti_Full_Project.m) ==========
% Run switches (to save time, you can run only one sweep)
if ~exist('run_rice_sweep', 'var'); run_rice_sweep = true; end
if ~exist('run_corr_sweep', 'var'); run_corr_sweep = true; end
if ~exist('run_doppler_sweep', 'var'); run_doppler_sweep = true; end
```

**Explanation:**
- **Line 22**: Section header for global parameters. These match the base project for consistency.
- **Line 24**: `if ~exist('run_rice_sweep', 'var')`: Checks if the variable `run_rice_sweep` exists in the workspace.
  - `~exist(..., 'var')` returns `true` if the variable does NOT exist.
  - If it doesn't exist, set it to `true` (default: run the Rice sweep).
  - If it already exists (from workspace or previous `clearvars -except`), keep its value.
- **Lines 25-26**: Same logic for correlation and Doppler sweeps. This allows selective execution: set `run_rice_sweep = false;` before running to skip the Rice sweep.

### Line 29: Fast Mode Switch

```matlab
if ~exist('fast_mode', 'var'); fast_mode = false; end
```

**Explanation:**
- Default is `false` (accurate mode: 500 frames, 2 dB SNR steps).
- If `fast_mode = true`, the script uses fewer frames (120) and larger SNR steps (4 dB) for faster debugging.

### Lines 31-35: Doppler Tracking Configuration

```matlab
% Doppler optimization: perform "periodic pilot tracking" for time-varying channels to avoid BER collapsing to 0.5
% - doppler_tracking=true: insert orthogonal pilots every doppler_data_seg_len data symbols and update (h1,h2) estimate
% - doppler_tracking=false: use only frame-head pilot estimation (high-speed Doppler will easily cause BER floor ~ 0.5)
if ~exist('doppler_tracking', 'var'); doppler_tracking = true; end
if ~exist('doppler_data_seg_len', 'var'); doppler_data_seg_len = 100; end   % Must be even; recommended 50~200
```

**Explanation:**
- **Lines 31-33**: Comments explain the two pilot modes:
  - `doppler_tracking = true`: Periodic pilots inserted every `doppler_data_seg_len` symbols to track fast-varying channels.
  - `doppler_tracking = false`: Single pilot at frame head only. Fast Doppler causes BER to approach 0.5 (random guessing).
- **Line 34**: Default is `true` (enable periodic tracking).
- **Line 35**: Default segment length is 100 symbols. Must be even (Alamouti decodes in pairs). Range 50-200 is recommended.

### Lines 37-44: System Parameters

```matlab
nb_data = 1000;             % Number of data symbols per frame (must be even for Alamouti pair encoding)
nb_pilot = 10;              % Number of pilot symbols (each antenna occupies nb_pilot time slots, total pilot overhead = 2*nb_pilot)
nb_bit_per_symb = 4;        % 16-QAM
rolloff = 0.5;
symb_rate = 100e6;
sps = 2;
span = 16;
fs = symb_rate * sps;
```

**Explanation:**
- **Line 37**: `nb_data = 1000`: Data symbols per frame. Must be even because Alamouti encodes symbols in pairs (s1, s2).
- **Line 38**: `nb_pilot = 10`: Pilot symbols per antenna. With orthogonal pilots, TX1 sends pilots in slots 1-10, TX2 in slots 11-20. Total overhead = 20 symbol slots.
- **Line 39**: `nb_bit_per_symb = 4`: 16-QAM modulation (4 bits per symbol).
- **Line 40**: `rolloff = 0.5`: Root-raised cosine (RRC) filter rolloff factor (0 to 1). 0.5 is a common choice balancing bandwidth and ISI.
- **Line 41**: `symb_rate = 100e6`: Symbol rate = 100 million symbols per second (100 Msps).
- **Line 42**: `sps = 2`: Samples per symbol. The signal is oversampled by 2x for pulse shaping.
- **Line 43**: `span = 16`: RRC filter span in symbols. The filter impulse response lasts 16 symbol periods.
- **Line 44**: `fs = symb_rate * sps`: Sampling frequency = 200 MHz (100 Msps × 2).

### Lines 46-47: Derived Parameters

```matlab
nb_bit = nb_data * nb_bit_per_symb;
g = raised_cosine(rolloff, span, sps, 'sqrt');
```

**Explanation:**
- **Line 46**: `nb_bit = 1000 × 4 = 4000` bits per frame.
- **Line 47**: `raised_cosine(...)`: Generates the RRC filter impulse response.
  - `'sqrt'`: Root-raised cosine (not full RC). TX and RX both use RRC; their cascade forms a full RC filter.
  - Returns a vector `g` of length `span*sps + 1 = 33` samples.

### Lines 49-56: BER Simulation Settings

```matlab
% BER simulation settings
if fast_mode
    Lsnr_dB = 0:4:20;
    nb_frame_ber = 120;
else
    Lsnr_dB = 0:2:20;
    nb_frame_ber = 500;
end
```

**Explanation:**
- **Line 50**: Check if fast mode is enabled.
- **Line 51**: Fast mode: SNR points at 0, 4, 8, 12, 16, 20 dB (6 points, 4 dB steps).
- **Line 52**: Fast mode: 120 Monte Carlo frames per SNR point (faster, less accurate).
- **Line 54**: Accurate mode: SNR points at 0, 2, 4, ..., 20 dB (11 points, 2 dB steps).
- **Line 55**: Accurate mode: 500 frames per SNR point (slower, more accurate BER curves).

### Line 59: Random Seed

```matlab
% For reproducibility (you can comment this out)
rng(1);
```

**Explanation:**
- `rng(1)`: Sets the random number generator seed to 1.
- **Why**: Ensures reproducible results. Every run produces identical random numbers (channels, noise, bits).
- **When to comment out**: For true randomness, comment this line. Useful for debugging and comparing results across runs.

### Lines 61-63: Pilot Generation

```matlab
% Generate pilot once (all sweeps reuse the same pilot for fair comparison)
bit_pilot = randi([0 1], nb_pilot * nb_bit_per_symb, 1);
symb_pilot = mapping_QAM(bit_pilot, nb_bit_per_symb, length(bit_pilot));
```

**Explanation:**
- **Line 61**: Comment explains why pilots are generated once: all three sweeps use the same pilot sequence for fair comparison.
- **Line 62**: `randi([0 1], ...)`: Generates random bits for pilots.
  - Size: `nb_pilot × nb_bit_per_symb = 10 × 4 = 40` bits.
  - `[0 1]`: Binary values.
  - `1`: Column vector.
- **Line 63**: `mapping_QAM(...)`: Maps 40 bits to 10 complex 16-QAM symbols.
  - Input: 40 bits.
  - Output: 10 symbols (column vector).
  - These symbols are known at the receiver for channel estimation.

### Lines 65-67: Console Output

```matlab
fprintf('=== Main_Alamouti_Extensions.m ===\n');
fprintf('16-QAM, nb_data=%d, nb_pilot=%d (orthogonal pilots => 2*nb_pilot), sps=%d, span=%d\n', nb_data, nb_pilot, sps, span);
fprintf('SNR sweep: %d points, Monte-Carlo frames per point: %d\n\n', length(Lsnr_dB), nb_frame_ber);
```

**Explanation:**
- **Line 65**: Prints script header.
- **Line 66**: Prints system parameters: modulation, data/pilot counts, oversampling, filter span.
- **Line 67**: Prints simulation settings: number of SNR points and frames per point.
- `\n`: Newline character. `\n\n`: Double newline for spacing.

---

## Extension 1: Rice (Rician) Channel Sweep

### Lines 69-78: Rice Sweep Setup

```matlab
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
```

**Explanation:**
- **Line 69**: Section header for Rice sweep.
- **Lines 72-76**: K-factor values to sweep (in dB):
  - Fast mode: K = 0 dB, 10 dB (2 points).
  - Full mode: K = 0 dB, 5 dB, 10 dB (3 points).
  - K = 0 dB is pure Rayleigh (no LOS). Higher K means stronger LOS.
- **Line 77**: `rho = 0`: No spatial correlation (h1 and h2 are independent).
- **Line 78**: `fdTs = 0`: Block fading (channel constant over entire frame).

### Line 80: BER Storage Initialization

```matlab
BER_Rice = zeros(length(K_dB_list), length(Lsnr_dB));
```

**Explanation:**
- Creates a 2D array to store BER results.
- Rows: one per K value.
- Columns: one per SNR point.
- Example: Fast mode → `[2 × 6]` matrix. Full mode → `[3 × 11]` matrix.

### Lines 82-104: Rice Sweep Execution

```matlab
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
```

**Explanation:**
- **Line 82**: Check if Rice sweep is enabled.
- **Line 83**: Print progress message.
- **Line 84**: Loop over each K value.
- **Line 85**: `K = 10^(K_dB_list(iK)/10)`: Convert K from dB to linear scale.
  - Example: K_dB = 10 → K = 10^(10/10) = 10.
- **Line 86**: `struct(...)`: Create channel options structure.
  - `'type', 'Rice'`: Rician channel.
  - `'K', K`: K-factor (linear).
  - `'rho', rho`: Spatial correlation (0).
  - `'fdTs', fdTs`: Normalized Doppler (0 = block fading).
- **Lines 87-89**: Call `sim_ber_alamouti()` to compute BER for all SNR points.
  - Returns a row vector (one BER per SNR).
  - Store in `BER_Rice(iK, :)` (row `iK`).
- **Line 90**: Print completion message for current K.
- **Line 93**: Create new figure window with title.
- **Line 94**: `hold on`: Keep previous plots when adding new ones.
- **Line 94**: `grid on`: Show grid lines.
- **Line 95**: Loop to plot each K curve.
- **Line 96**: `semilogy(...)`: Plot with logarithmic y-axis (BER spans many orders of magnitude).
  - `'-o'`: Line with circle markers.
  - `'LineWidth', 2`: Thicker lines for visibility.
  - `'MarkerSize', 7`: Marker size.
  - `'DisplayName', ...`: Legend label.
- **Lines 99-101**: Axis labels and title.
- **Line 102**: `legend(...)`: Show legend in southwest corner.
- **Line 103**: `ylim([1e-5 1])`: Set y-axis range (10⁻⁵ to 1).

---

## Extension 2: Spatial Correlation Sweep

### Lines 106-117: Correlation Sweep Setup

```matlab
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
```

**Explanation:**
- **Line 106**: Section header for correlation sweep.
- **Lines 109-113**: Correlation coefficient values:
  - Fast mode: ρ = 0, 0.9 (2 points).
  - Full mode: ρ = 0, 0.5, 0.9 (3 points).
  - ρ = 0: Independent channels (ideal). ρ = 0.9: Highly correlated (diversity loss).
- **Line 114**: `K = 0`: Pure Rayleigh (no LOS).
- **Line 115**: `fdTs = 0`: Block fading.

### Line 117: BER Storage

```matlab
BER_Rho = zeros(length(rho_list), length(Lsnr_dB));
```

**Explanation:**
- Similar to `BER_Rice`, stores BER for each ρ and SNR combination.

### Lines 119-141: Correlation Sweep Execution

```matlab
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
```

**Explanation:**
- **Line 119**: Check if correlation sweep is enabled.
- **Line 120**: Print progress (note `\n` for spacing).
- **Line 121**: Loop over each ρ value.
- **Line 122**: Get current ρ.
- **Line 123**: Create channel struct with `'type', 'Rayleigh'` (not Rice).
- **Lines 124-126**: Run BER simulation, store results.
- **Line 127**: Print completion.
- **Lines 130-141**: Plotting (similar to Rice sweep, but with `'-^'` markers and `\rho` in legend).

---

## Extension 3: Doppler Time-Varying Fading Sweep

### Lines 143-157: Doppler Sweep Setup

```matlab
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
```

**Explanation:**
- **Line 143**: Section header for Doppler sweep.
- **Lines 145-146**: Comments explain the challenge: Alamouti assumes constant channels over 2 time slots. Fast-varying channels break this assumption.
- **Lines 147-152**: Normalized Doppler values:
  - Fast mode: fdTs = 0, 0.005, 0.01, 0.02 (4 points).
  - Full mode: fdTs = 0, 0.002, 0.005, 0.01, 0.02 (5 points).
  - Note: Previously tested 0.2/0.5 caused BER ≈ 0.5 (complete failure), so smaller values are used.
- **Line 153**: `rho = 0`: No spatial correlation.
- **Line 154**: `K = 0`: Rayleigh channel.

### Line 157: BER Storage

```matlab
BER_Doppler = zeros(length(fdTs_list), length(Lsnr_dB));
```

**Explanation:**
- Stores BER for each fdTs and SNR combination.

### Lines 159-192: Doppler Sweep Execution

```matlab
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
```

**Explanation:**
- **Line 159**: Check if Doppler sweep is enabled.
- **Line 160**: Print progress message.
- **Lines 161-165**: Print tracking mode status.
- **Line 166**: Loop over each fdTs value.
- **Line 167**: Get current fdTs.
- **Line 168**: Create base channel struct.
- **Lines 169-173**: Add pilot mode to struct:
  - If `doppler_tracking = true`: `'periodic'` mode with segment length.
  - If `doppler_tracking = false`: `'single'` mode (frame-head only).
- **Lines 174-176**: Run BER simulation.
- **Line 177**: Print completion with AR(1) coefficient `a = J₀(2π·fdTs)` (used in channel generation).
- **Lines 180-191**: Plotting (similar to previous sweeps, with `'-s'` square markers).

### Lines 194-195: Completion Message

```matlab
fprintf('\n=== Done ===\n');
fprintf('You can now use these 3 figures directly in the "Extension/Discussion" section of your presentation.\n');
```

**Explanation:**
- Prints completion message and usage hint for presentation.

---

## Local Function: sim_ber_alamouti()

### Function Signature (Lines 200-202)

```matlab
function BER = sim_ber_alamouti(Lsnr_dB, nb_frame_ber, ...
    nb_data, nb_bit, nb_bit_per_symb, nb_pilot, symb_pilot, ...
    g, sps, span, chanOpt)
```

**Explanation:**
- **Inputs:**
  - `Lsnr_dB`: Vector of SNR values in dB (e.g., `[0 2 4 ... 20]`).
  - `nb_frame_ber`: Number of Monte Carlo frames per SNR point.
  - `nb_data`, `nb_bit`, `nb_bit_per_symb`: Frame parameters.
  - `nb_pilot`: Pilot symbols per antenna.
  - `symb_pilot`: Pre-generated pilot symbol sequence.
  - `g`: RRC filter impulse response.
  - `sps`, `span`: Oversampling and filter span.
  - `chanOpt`: Structure with channel parameters (`type`, `K`, `rho`, `fdTs`, optionally `pilot_mode`, `data_seg_len`).
- **Output:**
  - `BER`: Row vector of BER values, one per SNR point.

### Lines 204-225: Initialization and Pilot Mode Setup

```matlab
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
```

**Explanation:**
- **Line 204**: Initialize BER output vector.
- **Lines 207-211**: Check if `pilot_mode` exists in `chanOpt`. Default to `'single'` if not specified.
- **Line 213**: `strcmpi(...)`: Case-insensitive string comparison. Check if periodic mode.
- **Lines 214-216**: Error if `data_seg_len` is missing in periodic mode.
- **Line 217**: Extract segment length.
- **Lines 218-220**: Error if segment length is odd (Alamouti needs pairs).
- **Lines 221-223**: Error if `nb_data` is not divisible by `data_seg_len`.
- **Line 224**: Calculate number of segments: `nSeg = 1000 / 100 = 10` segments.

### Lines 227-229: SNR Loop Initialization

```matlab
for k = 1:length(Lsnr_dB)
    snr_lin = 10^(Lsnr_dB(k)/10);
    ber_acc = 0;
```

**Explanation:**
- **Line 227**: Loop over each SNR value.
- **Line 228**: Convert SNR from dB to linear: `snr_lin = 10^(SNR_dB/10)`.
  - Example: 10 dB → 10^(10/10) = 10 (linear).
- **Line 229**: Initialize BER accumulator for this SNR (sum over all frames).

### Lines 231-234: Frame Loop - Data Generation

```matlab
    for iFrm = 1:nb_frame_ber
        %% ===== TX: data generation + QAM mapping =====
        bits = randi([0 1], nb_bit, 1);
        syms_data = mapping_QAM(bits, nb_bit_per_symb, nb_bit);
```

**Explanation:**
- **Line 231**: Loop over Monte Carlo frames.
- **Line 233**: `randi([0 1], ...)`: Generate random bits for this frame.
  - Size: `nb_bit × 1` (e.g., 4000 bits).
- **Line 234**: `mapping_QAM(...)`: Map bits to 16-QAM symbols.
  - Input: 4000 bits.
  - Output: 1000 complex symbols (column vector).

### Lines 236-246: Alamouti Encoding

```matlab
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
```

**Explanation:**
- **Line 237**: Section header for Alamouti encoding.
- **Line 238**: `s1 = syms_data(1:2:end)`: Extract odd-indexed symbols (1, 3, 5, ...).
  - Result: 500 symbols.
- **Line 239**: `s2 = syms_data(2:2:end)`: Extract even-indexed symbols (2, 4, 6, ...).
  - Result: 500 symbols.
- **Line 240**: `scale = 1/sqrt(2)`: Power normalization factor. Ensures total transmit power is constant.
- **Line 242**: Initialize TX1 data vector (1000 elements).
- **Line 243**: Initialize TX2 data vector (1000 elements).
- **Line 244**: `tx1_data(1:2:end) = s1 * scale`: TX1 sends s1/√2 at odd time slots (1, 3, 5, ...).
- **Line 245**: `tx2_data(1:2:end) = s2 * scale`: TX2 sends s2/√2 at odd time slots.
- **Line 246**: `tx1_data(2:2:end) = -conj(s2) * scale`: TX1 sends -s₂*/√2 at even time slots (2, 4, 6, ...).
- **Line 247**: `tx2_data(2:2:end) = conj(s1) * scale`: TX2 sends s₁*/√2 at even time slots.

**Alamouti Encoding Matrix:**
```
Time Slot | TX1          | TX2
----------|--------------|--------------
t=1       | s1/√2        | s2/√2
t=2       | -s2*/√2      | s1*/√2
```

### Lines 248-268: Pilot Insertion

```matlab
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
```

**Explanation:**
- **Line 249**: `pilot_len = nb_pilot`: 10 symbols.
- **Line 250**: `pilot_tx1 = [symb_pilot; zeros(pilot_len, 1)]`: TX1 pilot sequence.
  - First 10 slots: pilot symbols.
  - Next 10 slots: zeros (TX1 silent while TX2 sends pilots).
  - Total: 20 symbol slots.
- **Line 251**: `pilot_tx2 = [zeros(pilot_len, 1); symb_pilot]`: TX2 pilot sequence.
  - First 10 slots: zeros (TX2 silent while TX1 sends pilots).
  - Next 10 slots: pilot symbols.
  - Total: 20 symbol slots.
- **Line 253**: Check pilot mode.
- **Lines 254-256**: **Single mode**: Frame structure = `[pilots (20 slots) | data (1000 slots)]`.
  - Total: 1020 symbol slots.
- **Lines 257-267**: **Periodic mode**: Frame structure = `[pilots | data_seg]` repeated `nSeg` times.
  - **Line 259**: Initialize empty frame vectors.
  - **Line 260**: Loop over segments.
  - **Lines 261-262**: Calculate data segment indices.
    - Segment 1: idx1=1, idx2=100.
    - Segment 2: idx1=101, idx2=200.
    - etc.
  - **Lines 263-264**: Append `[pilots | segment_data]` to frame.
    - `%#ok<AGROW>`: Suppress MATLAB warning about array growth in loop (acceptable here).
  - **Line 267**: Total frame length = `nSeg × (20 pilots + 100 data) = 10 × 120 = 1200` symbols.

### Lines 270-272: Pulse Shaping

```matlab
        %% ===== Pulse shaping =====
        sig_tx1 = convolution_TX(frame_tx1, g, sps);
        sig_tx2 = convolution_TX(frame_tx2, g, sps);
```

**Explanation:**
- **Line 271**: `convolution_TX(...)`: Apply RRC pulse shaping to TX1 frame.
  - Upsamples by `sps` (inserts zeros).
  - Convolves with filter `g`.
  - Output: Sample-rate signal (length ≈ `frame_len_symb × sps`).
- **Line 272**: Same for TX2.

### Lines 274-276: Channel Application

```matlab
        %% ===== Channel (2x1 MISO) =====
        [h1_samp, h2_samp] = gen_miso_2x1_channel_samples(length(sig_tx1), sps, span, chanOpt);
        r_nl = h1_samp .* sig_tx1 + h2_samp .* sig_tx2;
```

**Explanation:**
- **Line 275**: `gen_miso_2x1_channel_samples(...)`: Generate time-varying channel coefficients at sample rate.
  - Input: Number of samples needed, `sps`, `span` (for padding), `chanOpt`.
  - Output: `h1_samp`, `h2_samp` (column vectors, one coefficient per sample).
- **Line 276**: Apply 2×1 MISO channel: `r = h₁·s₁ + h₂·s₂`.
  - `.*`: Element-wise multiplication.
  - `r_nl`: Received signal (no noise yet).

### Lines 278-282: AWGN Addition

```matlab
        %% ===== AWGN =====
        p_ala = sum(abs(sig_tx1).^2 + abs(sig_tx2).^2) / length(sig_tx1) * sps;
        n_var = p_ala / snr_lin;
        noise = sqrt(n_var/2) * (randn(size(r_nl)) + 1i*randn(size(r_nl)));
        r = r_nl + noise;
```

**Explanation:**
- **Line 279**: Calculate average transmit power `p_ala`:
  - `abs(sig_tx1).^2 + abs(sig_tx2).^2`: Power per sample (both antennas).
  - `sum(...)`: Total power.
  - `/ length(sig_tx1)`: Average power per sample.
  - `* sps`: Adjust for oversampling (power normalization).
- **Line 280**: Calculate noise variance: `n_var = P_signal / SNR`.
  - For complex AWGN, variance per real/imaginary component = `n_var/2`.
- **Line 281**: Generate complex AWGN:
  - `randn(...)`: Real part ~ N(0,1).
  - `1i*randn(...)`: Imaginary part ~ N(0,1).
  - `sqrt(n_var/2)`: Scale to desired variance.
- **Line 282**: Add noise to received signal.

### Lines 284-286: Matched Filter and Sampling

```matlab
        %% ===== RX matched filter + sampling =====
        symb_rx = convolution_RX(r, g, sps);
        symb_rx = symb_rx(span+1 : span+frame_len_symb);
```

**Explanation:**
- **Line 285**: `convolution_RX(...)`: Apply matched filter (RRC) and downsample.
  - Convolves received signal with `g`.
  - Downsamples by `sps` (symbol-rate output).
- **Line 286**: Crop to remove filter delay:
  - First `span` symbols are filter transients (discard).
  - Keep next `frame_len_symb` symbols (actual frame).

### Lines 288-343: Channel Estimation and Alamouti Decoding

#### Single Pilot Mode (Lines 290-311)

```matlab
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
```

**Explanation:**
- **Line 289**: Initialize decoded symbol vector.
- **Line 290**: Check if single pilot mode.
- **Lines 292-293**: Extract received pilots:
  - `pilot_rx1`: Slots 1-10 (TX1 pilots, TX2 silent).
  - `pilot_rx2`: Slots 11-20 (TX2 pilots, TX1 silent).
- **Lines 295-296**: **Least Squares channel estimation**:
  - `pilot_rx1 ./ symb_pilot`: Element-wise division (remove known pilot modulation).
  - `sum(...) / pilot_len`: Average over 10 pilots → estimate of h₁.
  - Same for h₂.
- **Line 298**: Extract data portion (slots 21 to end).
- **Lines 299-300**: Extract Alamouti pairs:
  - `y1`: Odd slots (1st time slot of each pair).
  - `y2`: Even slots (2nd time slot of each pair).
- **Line 301**: `y2c = conj(y2)`: Conjugate of y₂ (needed for decoding).
- **Lines 303-304**: **Alamouti decoding**:
  - `z1 = (1/√2) · (h₁*·y₁ + h₂·y₂*)`: Linear combination to recover s₁.
  - `z2 = (1/√2) · (h₂*·y₁ - h₁·y₂*)`: Linear combination to recover s₂.
- **Line 306**: `alpha = (|h₁|² + |h₂|²) / 2`: Normalization factor (channel gain).
- **Lines 307-308**: **Equalization**: Divide by `alpha` to remove channel gain.
- **Lines 310-311**: Store decoded symbols in `rec` vector (interleaved: s1, s2, s1, s2, ...).

#### Periodic Pilot Mode (Lines 312-342)

```matlab
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
```

**Explanation:**
- **Line 314**: `blkLen = 2*pilot_len + data_seg_len`: Block length = 20 + 100 = 120 symbols.
- **Line 315**: Loop over each segment.
- **Line 316**: Calculate block start index:
  - Segment 1: blkStart = 1.
  - Segment 2: blkStart = 121.
  - etc.
- **Lines 318-319**: Extract pilots for this segment (same as single mode, but per segment).
- **Lines 321-322**: Estimate channel for this segment (fresh estimate every 100 data symbols).
- **Lines 324-325**: Extract data block for this segment.
- **Lines 327-340**: Decode this segment's data (same Alamouti decoding as single mode).
- **Lines 338-341**: Store decoded symbols in correct positions in `rec`:
  - `recIdx1`: Start index for this segment.
  - `recIdx2`: End index for this segment.
  - Store s1 at odd indices, s2 at even indices.

### Lines 345-351: Hard Decision and BER Calculation

```matlab
        %% ===== Hard decision + demap =====
        symb_hat = symbol_estimation_QAM(rec, nb_bit_per_symb, nb_data);
        bits_hat = demapping_QAM(symb_hat, nb_bit_per_symb, nb_data);

        %% ===== BER accumulate =====
        [~, ber_frame] = biterr(bits, bits_hat);
        ber_acc = ber_acc + ber_frame;
    end
```

**Explanation:**
- **Line 346**: `symbol_estimation_QAM(...)`: Minimum-distance detection.
  - Input: Noisy decoded symbols `rec`.
  - Output: Hard-decided symbols (nearest constellation point).
- **Line 347**: `demapping_QAM(...)`: Convert symbols back to bits.
  - Input: Hard-decided symbols.
  - Output: Estimated bit sequence.
- **Line 350**: `biterr(bits, bits_hat)`: Compare transmitted and received bits.
  - Returns: `[num_errors, ber]`.
  - `~`: Ignore number of errors (not needed).
  - `ber_frame`: BER for this frame (errors / total_bits).
- **Line 351**: Accumulate BER over frames.

### Lines 353-354: Average BER and Return

```matlab
    BER(k) = ber_acc / nb_frame_ber;
end
end
```

**Explanation:**
- **Line 353**: Average BER over all frames: `BER(k) = total_BER / nb_frame_ber`.
- **Line 354**: End of function.

---

## Local Function: gen_miso_2x1_channel_samples()

### Function Signature and Header (Lines 358-363)

```matlab
function [h1_samp, h2_samp] = gen_miso_2x1_channel_samples(nb_samp, sps, span, chanOpt)
% Output h1_samp/h2_samp: column vectors of length nb_samp (channel coefficient for each sample)
%
% Generation approach:
% - First generate channel sequence H_symb at "symbol rate" (length ~ nb_samp/sps)
% - Then repeat each symbol's channel coefficient sps times to expand to sample points
```

**Explanation:**
- **Inputs:**
  - `nb_samp`: Number of samples needed (sample-rate length).
  - `sps`: Samples per symbol (oversampling factor).
  - `span`: Filter span (used for padding).
  - `chanOpt`: Channel configuration structure.
- **Outputs:**
  - `h1_samp`, `h2_samp`: Channel coefficients at sample rate (column vectors).
- **Approach**: Generate at symbol rate first, then expand to sample rate.

### Lines 365-368: Spatial Correlation Setup

```matlab
rho = chanOpt.rho;
if abs(rho) >= 1
    error('rho must satisfy |rho|<1 to ensure covariance matrix is positive definite.');
end
```

**Explanation:**
- **Line 365**: Extract correlation coefficient.
- **Lines 366-368**: Validate `rho`: Must be in `(-1, 1)` for positive definite covariance matrix.

### Lines 370-371: Symbol-Rate Length Calculation

```matlab
% Number of "symbol intervals" needed: use ceil(nb_samp/sps) + span for padding to avoid boundary issues
nb_symb = ceil(nb_samp / sps) + span;
```

**Explanation:**
- **Line 371**: Calculate number of symbol intervals needed:
  - `ceil(nb_samp / sps)`: Minimum symbols to cover `nb_samp` samples.
  - `+ span`: Add padding to avoid boundary effects when expanding to sample rate.

### Lines 373-385: Doppler (AR(1) Coefficient) Calculation

```matlab
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
```

**Explanation:**
- **Line 374**: Extract normalized Doppler frequency.
- **Lines 375-377**: Validate `fdTs >= 0`.
- **Lines 379-384**: Calculate AR(1) coefficient:
  - **Line 380**: `fdTs == 0`: Block fading (no time variation).
    - `a = 1`: Perfect correlation (channel constant).
  - **Line 382**: `fdTs > 0`: Time-varying channel.
    - `besselj(0, 2*pi*fdTs)`: Zeroth-order Bessel function (Jakes autocorrelation at lag 1).
    - This is the correlation between adjacent symbol intervals.
  - **Lines 383-384**: Numerical safety: clamp `a` to `[-1, 1]` to ensure `sqrt(1-a²)` is real.

### Lines 387-389: Spatial Correlation Matrix and Cholesky Decomposition

```matlab
% Antenna correlation covariance matrix (2x2)
R = [1 rho; rho 1];
L = chol(R, 'lower');
```

**Explanation:**
- **Line 388**: Build 2×2 covariance matrix:
  ```
  R = [1   ρ]
      [ρ   1]
  ```
  - Diagonal = 1 (unit variance per antenna).
  - Off-diagonal = ρ (correlation between antennas).
- **Line 389**: `chol(R, 'lower')`: Cholesky decomposition.
  - Returns lower triangular matrix `L` such that `R = L·L'`.
  - Used to generate correlated random variables from independent ones.

### Lines 391-404: Generate NLOS (Rayleigh) Component with Time Correlation

```matlab
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
```

**Explanation:**
- **Line 392**: Generate independent complex Gaussian:
  - `randn(2, nb_symb)`: Real part ~ N(0,1), size 2×nb_symb.
  - `1i*randn(...)`: Imaginary part ~ N(0,1).
  - `/ sqrt(2)`: Normalize so each component has variance 1/2 → `E[|h|²] = 1`.
- **Line 393**: `W = L * U`: Apply Cholesky to create spatially correlated channels.
  - `W(1,:)`, `W(2,:)`: Correlated (correlation = ρ).
- **Line 395**: Initialize NLOS channel matrix (2 antennas × nb_symb symbols).
- **Line 396**: `H_nlos(:, 1) = W(:, 1)`: Initialize first symbol with spatially correlated values.
- **Lines 397-399**: **Block fading** (`a = 1`):
  - `repmat(...)`: Repeat first symbol's channel for all symbols (constant channel).
- **Lines 400-403**: **Time-varying** (`a < 1`):
  - **Line 400**: `b = sqrt(1 - a²)`: Innovation coefficient (ensures unit variance).
  - **Line 401**: Loop over symbols.
  - **Line 402**: **AR(1) model**: `h[n] = a·h[n-1] + b·w[n]`.
    - `a·h[n-1]`: Correlated with previous symbol.
    - `b·w[n]`: Innovation (new random component).
    - This generates time-correlated fading with Jakes autocorrelation.

### Lines 406-421: Apply Channel Type (Rayleigh or Rice)

```matlab
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
```

**Explanation:**
- **Line 407**: Switch based on channel type.
- **Lines 408-409**: **Rayleigh**: Use NLOS component directly.
- **Lines 410-418**: **Rice**:
  - **Line 411**: Extract K-factor (linear scale).
  - **Lines 412-414**: Validate `K >= 0`.
  - **Line 416**: `phi = 2*pi*rand(2, 1)`: Random phase for each antenna (2×1 vector).
  - **Line 417**: `H_los = exp(1i*phi) * ones(1, nb_symb)`: LOS component.
    - `exp(1i*phi)`: Complex exponential (unit magnitude, random phase).
    - `* ones(1, nb_symb)`: Replicate across all symbols (LOS is constant).
    - Result: 2×nb_symb matrix.
  - **Line 418**: **Rician channel formula**:
    ```
    H = √(K/(1+K)) · H_LOS + √(1/(1+K)) · H_NLOS
    ```
    - First term: LOS component (weighted by K).
    - Second term: NLOS component (weighted by 1).
    - Total power: `K/(1+K) + 1/(1+K) = 1` (normalized).
- **Line 420**: Error for unknown channel type.

### Lines 423-428: Expand to Sample Rate

```matlab
% Expand to sample points: repeat each symbol sps times
H_samp = kron(H, ones(1, sps));    % 2 x (nb_symb*sps)
H_samp = H_samp(:, 1:nb_samp);

h1_samp = H_samp(1, :).';
h2_samp = H_samp(2, :).';
```

**Explanation:**
- **Line 424**: `kron(H, ones(1, sps))`: Kronecker product to expand symbol-rate to sample-rate.
  - `H`: 2×nb_symb (symbol-rate channels).
  - `ones(1, sps)`: Row vector of ones (e.g., `[1 1]` for `sps=2`).
  - Result: Each symbol's channel coefficient is repeated `sps` times.
  - Output size: 2×(nb_symb×sps).
- **Line 425**: Crop to exact length needed: `H_samp(:, 1:nb_samp)`.
- **Lines 427-428**: Extract channels for each antenna and transpose to column vectors.

---

## Summary

This document has explained every line of code in `Main_Alamouti_Extensions.m`, covering:

1. **Initialization**: Variable clearing, parameter setup, pilot generation.
2. **Three Extension Sweeps**: Rice K-factor, spatial correlation, Doppler time-varying fading.
3. **Core Simulation Function**: `sim_ber_alamouti()` - complete transmit/receive chain with Alamouti encoding/decoding.
4. **Channel Generation Function**: `gen_miso_2x1_channel_samples()` - spatial correlation, time correlation (AR(1)), and channel type (Rayleigh/Rice).

Each line's purpose, mathematical meaning, and role in the overall simulation has been detailed to provide a comprehensive understanding of the codebase.

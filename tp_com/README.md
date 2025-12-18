# Project 3: Alamouti STBC MISO Simulation

## 📋 Overview

This project implements a complete **Alamouti Space-Time Block Code (STBC)** simulation for a **2×1 MISO** (Multiple-Input Single-Output) system. The implementation is based on the signal processing framework from `main_5_Signal_Space_Simulation_with_Channel.m` and follows the standard Alamouti decoding formulas from **Slides 24-25** of the MIMO Processing course.

### Key Features

- ✅ **16-QAM modulation** (consistent with main5 baseline)
- ✅ **Pilot-based channel estimation** (LS method)
- ✅ **Standard Alamouti encoding/decoding** (Slides 24-25)
- ✅ **BER performance comparison**: SISO vs Alamouti 2×1
- ✅ **Visualization**: TX/RX constellations, spectrum, BER curves

---

## 📚 Theoretical Background

### Alamouti Space-Time Block Code

The Alamouti scheme is a transmit diversity technique for **2 TX antennas and 1 RX antenna** that achieves **full diversity order (2)** with a simple linear decoder.

#### Encoding (Slide 24)

For two consecutive symbols $(s_1, s_2)$, the transmission matrix is:

| Time Slot | Antenna 1                   | Antenna 2                  |
| --------- | --------------------------- | -------------------------- |
| $t_1$   | $\frac{s_1}{\sqrt{2}}$    | $\frac{s_2}{\sqrt{2}}$   |
| $t_2$   | $\frac{-s_2^*}{\sqrt{2}}$ | $\frac{s_1^*}{\sqrt{2}}$ |

The $\frac{1}{\sqrt{2}}$ factor ensures **power normalization** (total TX power = 1).

#### Received Signal

At the single RX antenna:

```
y₁ = h₁·x₁₁ + h₂·x₂₁ + n₁  (time slot 1)
y₂ = h₁·x₁₂ + h₂·x₂₂ + n₂  (time slot 2)
```

Where $h_1$ and $h_2$ are the channel coefficients from TX1 and TX2 respectively.

#### Decoding (Slide 25)

The Alamouti decoder computes:

$$
z_1 = \frac{1}{\sqrt{2}} \left( h_1^* y_1 + h_2 y_2^* \right)
$$

$$
z_2 = \frac{1}{\sqrt{2}} \left( h_2^* y_1 - h_1 y_2^* \right)
$$

After equalization by $\alpha = \frac{|h_1|^2 + |h_2|^2}{2}$:

$$
\hat{s}_1 = \frac{z_1}{\alpha}, \quad \hat{s}_2 = \frac{z_2}{\alpha}
$$

### Diversity Gain

- **SISO (Rayleigh)**: Diversity order = 1, BER slope ~ $\frac{1}{\text{SNR}}$
- **Alamouti 2×1**: Diversity order = 2, BER slope ~ $\frac{1}{\text{SNR}^2}$

This results in approximately **3-5 dB SNR gain** at moderate-to-high SNR.

---

## 📁 File Structure

```
tp_com/
├── Main_Alamouti_Full_Project.m    # 🎯 Main simulation script (Project 3)
├── main_5_Signal_Space_Simulation_with_Channel.m  # Reference SISO simulation
├── SNR_test.m                      # SNR sweep for BER analysis
│
├── mapping_QAM.m                   # QAM symbol mapping
├── demapping_QAM.m                 # QAM symbol demapping
├── symbol_estimation_QAM.m         # Symbol decision (hard decision)
│
├── raised_cosine.m                 # Root Raised Cosine filter design
├── convolution_TX.m                # TX pulse shaping (upsampling + filtering)
├── convolution_RX.m                # RX matched filtering (filtering + downsampling)
│
├── generate_channel.m              # Rayleigh/Rice channel generation
│
├── Unit 4 - 3 - MIMO Processing.pdf    # Course slides
├── MU5EEF08 - Project ideas (2).pdf    # Project requirements
└── README.md                       # This file
```

---

## 🔧 Dependencies

### MATLAB Toolboxes Required

- **Communications Toolbox** (for `biterr`, `berfading` functions)
- **Signal Processing Toolbox** (optional, for advanced filtering)

### Custom Functions (Provided)

All custom functions are included in the `tp_com/` folder:

- `mapping_QAM.m` / `demapping_QAM.m`
- `symbol_estimation_QAM.m`
- `raised_cosine.m`
- `convolution_TX.m` / `convolution_RX.m`
- `generate_channel.m`

---

## 🚀 Usage

### Quick Start

1. Open MATLAB and navigate to the `tp_com/` directory
2. Run the main script:
   ```matlab
   Main_Alamouti_Full_Project
   ```

### Expected Output

The script will:

1. **Part A**: Generate visualization figures (spectrum, constellations)
2. **Part B**: Run Monte Carlo BER simulation (~1-2 minutes)
3. Display BER comparison curves

### Console Output Example

```
Exécution Partie A: Visualisation (SNR = 20dB, 16-QAM)...
Canal réel:  h1 = 0.066+0.838i, h2 = 0.253+0.041i
Canal estimé: h1 = 0.046+0.843i, h2 = 0.240+0.037i

Exécution Partie B: Simulation BER (16-QAM, estimation pilotes)...
Comparaison: SISO (baseline main5) vs Alamouti 2x1
SNR  0 dB: BER_SISO = 3.36e-01, BER_Alamouti = 3.25e-01
SNR  2 dB: BER_SISO = 2.93e-01, BER_Alamouti = 2.75e-01
...
SNR 20 dB: BER_SISO = 2.71e-02, BER_Alamouti = 4.28e-03

=== Simulation terminée ===
```

---

## 📊 Code Structure

### Part A: Single Frame Visualization

```matlab
%% PARTIE A: Visualisation d'une trame
```

| Step | Description                    | Code Section                         |
| ---- | ------------------------------ | ------------------------------------ |
| 1    | Generate random data bits      | `bit_TX = randi([0 1], nb_bit, 1)` |
| 2    | QAM mapping                    | `symb_TX_data = mapping_QAM(...)`  |
| 3    | Alamouti encoding              | Lines 36-53                          |
| 4    | Add orthogonal pilots          | Lines 55-66                          |
| 5    | Pulse shaping (RRC)            | `convolution_TX(...)`              |
| 6    | Rayleigh channel               | `H1*sig_TX1 + H2*sig_TX2`          |
| 7    | Add AWGN noise                 | Lines 86-90                          |
| 8    | Matched filtering              | `convolution_RX(...)`              |
| 9    | Pilot-based channel estimation | Lines 97-106                         |
| 10   | Alamouti decoding              | Lines 124-143                        |
| 11   | Plot constellations            | Figure 2                             |

### Part B: BER Monte Carlo Simulation

```matlab
%% PARTIE B: Analyse de Performance BER
```

| Parameter          | Value                  |
| ------------------ | ---------------------- |
| Number of frames   | 500                    |
| SNR range          | 0:2:20 dB              |
| Modulation         | 16-QAM                 |
| Channel            | Rayleigh (flat fading) |
| Channel estimation | Pilot-based LS         |

#### SISO Baseline (main5)

- Frame structure: `[Pilots | Data]`
- Channel estimation: $\hat{h} = \frac{1}{N_p} \sum_{i=1}^{N_p} \frac{y_i}{p_i}$
- Equalization: $\hat{s} = y / \hat{h}$ (ZF)

#### Alamouti 2×1

- Frame structure: `[Orthogonal Pilots | Alamouti Data]`
- Channel estimation: Separate $\hat{h}_1$ and $\hat{h}_2$ from orthogonal pilot slots
- Decoding: Standard Alamouti formula (Slide 25)

---

## 📈 Results

### Figure 1: TX Spectrum

Shows the **Root Raised Cosine** filtered spectrum with:

- Bandwidth ≈ $(1 + \beta) \times R_s = 150$ MHz
- Roll-off factor $\beta = 0.5$
- Symbol rate $R_s = 100$ Msym/s

### Figure 2: Constellation Diagrams

| Subplot           | Description                                             |
| ----------------- | ------------------------------------------------------- |
| 1. TX Symbols     | Standard 16-QAM constellation at ±1, ±3               |
| 2. RX Raw         | Distorted cloud (superposition of$h_1 x_1 + h_2 x_2$) |
| 3. After Decoding | Recovered 16-QAM constellation                          |

### Figure 3: BER Curves

| Curve            | Description                       |
| ---------------- | --------------------------------- |
| Blue circles     | SISO simulation (main5 baseline)  |
| Orange triangles | Alamouti 2×1 simulation          |
| Black dashed     | SISO theory (Rayleigh, div=1)     |
| Red dashed       | Alamouti theory (Rayleigh, div=2) |

**Key Observations:**

- Simulation curves closely match theoretical predictions
- Alamouti provides ~5-8 dB gain at high SNR
- Steeper slope for Alamouti (diversity order = 2)

---

## ⚙️ Parameters

### Simulation Parameters (Configurable)

```matlab
nb_data = 1000;          % Data symbols per frame
nb_pilot = 10;           % Pilot symbols
nb_bit_per_symb = 4;     % 16-QAM
rolloff = 0.5;           % RRC roll-off
symb_rate = 100e6;       % Symbol rate (Hz)
sps = 2;                 % Samples per symbol
span = 16;               % Filter span (symbols)
nb_frame_ber = 500;      % Monte Carlo iterations
Lsnr_dB = 0:2:20;        % SNR range (dB)
```

### Channel Model

- **Type**: Rayleigh flat fading
- **Generation**: `h = (randn + 1i*randn)/sqrt(2)`
- **Normalization**: $\mathbb{E}[|h|^2] = 1$

---

## 🔬 Comparison with main5

| Aspect             | main5 (SISO)    | This Project (Alamouti)  |
| ------------------ | --------------- | ------------------------ |
| TX antennas        | 1               | 2                        |
| RX antennas        | 1               | 1                        |
| Modulation         | 16-QAM          | 16-QAM                   |
| Channel estimation | Pilot LS        | Pilot LS (orthogonal)    |
| Diversity order    | 1               | 2                        |
| Pilot overhead     | $N_p$ symbols | $2 \times N_p$ symbols |

---

## 📖 References

1. **Alamouti, S.M.** (1998). "A simple transmit diversity technique for wireless communications." *IEEE Journal on Selected Areas in Communications*, 16(8), 1451-1458.
2. **Course Slides**: Unit 4 - 3 - MIMO Processing.pdf (Slides 24-25)
3. **Project Requirements**: MU5EEF08 - Project ideas (2).pdf
4. **MATLAB Documentation**:

   - [berfading](https://www.mathworks.com/help/comm/ref/berfading.html)
   - [biterr](https://www.mathworks.com/help/comm/ref/biterr.html)

---

## 👤 Author

**Course**: MU5EES08 - Advanced Physical Layer Concepts in 5G
**University**: Sorbonne Université
**Instructor**: Prof. Julien Sarrazin
**Semester**: Sept.-Dec. 2024

---

## 📝 License

This project is for educational purposes as part of the MU5EES08 course.

---

## 🔄 Version History

| Version | Date     | Changes                                              |
| ------- | -------- | ---------------------------------------------------- |
| 1.0     | Dec 2025 | Initial implementation with QPSK                     |
| 2.0     | Dec 2025 | Updated to 16-QAM, pilot estimation, French comments |
| 2.1     | Dec 2025 | Fixed Alamouti scaling formula (Slide 25 standard)   |

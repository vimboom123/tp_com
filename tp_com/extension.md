# Alamouti STBC Extensions: Rice K-Factor, Spatial Correlation, Doppler Time-Varying Fading

This document provides a detailed explanation of the **physical principles, mathematical models, and simulation result analysis** for the three extension directions in `Main_Alamouti_Extensions.m`.

---

## 1. Background: Alamouti Coding Review

### 1.1 System Model

Alamouti is a 2×1 MISO (2 transmit antennas, 1 receive antenna) Space-Time Block Code (STBC). During two consecutive time slots, the transmitter encodes as follows:

| Time Slot | TX1 Sends | TX2 Sends |
|-----------|-----------|-----------|
| t=1 | $s_1 / \sqrt{2}$ | $s_2 / \sqrt{2}$ |
| t=2 | $-s_2^* / \sqrt{2}$ | $s_1^* / \sqrt{2}$ |

The $1/\sqrt{2}$ factor is for power normalization, ensuring constant total transmit power.

### 1.2 Received Signal

Assuming channel coefficients $h_1, h_2$ remain constant during the two time slots (**block fading assumption**), the received signals are:

$$
y_1 = \frac{1}{\sqrt{2}}(h_1 s_1 + h_2 s_2) + n_1
$$

$$
y_2 = \frac{1}{\sqrt{2}}(-h_1 s_2^* + h_2 s_1^*) + n_2
$$

### 1.3 Alamouti Decoding

The original symbols are recovered through linear combination:

$$
z_1 = \frac{1}{\sqrt{2}}(h_1^* y_1 + h_2 y_2^*) = \frac{|h_1|^2 + |h_2|^2}{2} s_1 + \tilde{n}_1
$$

$$
z_2 = \frac{1}{\sqrt{2}}(h_2^* y_1 - h_1 y_2^*) = \frac{|h_1|^2 + |h_2|^2}{2} s_2 + \tilde{n}_2
$$

Final equalization:

$$
\hat{s}_i = \frac{z_i}{\alpha}, \quad \alpha = \frac{|h_1|^2 + |h_2|^2}{2}
$$

**Key Point**: Alamouti achieves **full diversity gain (diversity order = 2)** with only linear decoding complexity.

---

## 2. Extension 1: Rice (Rician) Channel Model

### 2.1 Physical Background

Real wireless channels are not purely Rayleigh fading. When a **Line-of-Sight (LOS) path** exists, the channel follows a Rician distribution:

- **Rayleigh fading**: No LOS, signal consists entirely of scattered paths
- **Rician fading**: LOS component + scattered components

### 2.2 Mathematical Model

Rician channel coefficient:

$$
h = \sqrt{\frac{K}{1+K}} \cdot h_{\text{LOS}} + \sqrt{\frac{1}{1+K}} \cdot h_{\text{NLOS}}
$$

Where:
- $K$ = Rice factor (ratio of LOS power to scatter power), typically expressed in dB
- $h_{\text{LOS}} = e^{j\phi}$: Fixed-phase LOS component
- $h_{\text{NLOS}} \sim \mathcal{CN}(0, 1)$: Complex Gaussian scattered component

### 2.3 Physical Meaning of K-Factor

| K Value | Scenario |
|---------|----------|
| K = 0 (−∞ dB) | Pure Rayleigh, no LOS (dense urban, indoor NLOS) |
| K = 1 (0 dB) | LOS and scatter power are equal |
| K = 10 (10 dB) | Strong LOS (open areas, satellite communications) |
| K → ∞ | Pure AWGN channel (no fading) |

### 2.4 Effect on BER

- **Higher K leads to lower BER**: LOS component provides stable signal, reducing deep fade probability
- However, Alamouti's diversity gain is less effective under Rician channels since the channel is already relatively stable

### 2.5 Code Implementation

```matlab
% Generate Rician channel
H_los = exp(1i * 2*pi*rand(2,1)) * ones(1, nb_symb);  % Random phase LOS
H = sqrt(K/(1+K)) * H_los + sqrt(1/(1+K)) * H_nlos;
```

---

## 3. Extension 2: Transmit Antenna Spatial Correlation

### 3.1 Physical Background

Ideal Alamouti assumes $h_1$ and $h_2$ are **independent and identically distributed (i.i.d.)**. However, in practice:

- **Insufficient antenna spacing**: When antenna spacing < λ/2, channels become highly correlated
- **Limited scatterer distribution**: Few scatterers around transmitter, small angular spread

### 3.2 Mathematical Model

The channel coefficients of two transmit antennas follow a joint complex Gaussian distribution:

$$
\begin{bmatrix} h_1 \\ h_2 \end{bmatrix} \sim \mathcal{CN}\left(\mathbf{0}, \mathbf{R}\right)
$$

Where the covariance matrix is:

$$
\mathbf{R} = \begin{bmatrix} 1 & \rho \\ \rho^* & 1 \end{bmatrix}
$$

$\rho \in [0, 1)$ is the correlation coefficient:
- $\rho = 0$: Completely independent (ideal case)
- $\rho \to 1$: Highly correlated (diversity gain lost)

### 3.3 Generating Correlated Channels

Using Cholesky decomposition:

$$
\mathbf{R} = \mathbf{L} \mathbf{L}^H
$$

```matlab
L = chol(R, 'lower');
U = (randn(2, N) + 1i*randn(2, N)) / sqrt(2);  % Independent CN(0,1)
H = L * U;  % Correlated channels
```

### 3.4 Effect on BER

- **Higher ρ leads to higher BER**
- Physical explanation: When $\rho \to 1$, $h_1 \approx h_2$, the Alamouti matrix becomes **rank-deficient**, diversity gain degrades from 2 to nearly 1
- Limiting case $\rho = 1$: Complete loss of diversity, performance approaches single-antenna SISO

### 3.5 Theoretical Analysis

Effective diversity order is approximately:

$$
d_{\text{eff}} \approx 2(1 - |\rho|^2)
$$

When $\rho = 0.9$, $d_{\text{eff}} \approx 0.38$, diversity gain is almost completely lost.

---

## 4. Extension 3: Doppler Time-Varying Fading

### 4.1 Physical Background

When the transmitter or receiver moves, channel coefficients vary with time. Doppler shift:

$$
f_D = \frac{v \cdot f_c}{c} \cos\theta
$$

Where:
- $v$: Moving speed
- $f_c$: Carrier frequency
- $c$: Speed of light
- $\theta$: Angle of arrival

### 4.2 Normalized Doppler Frequency

Define normalized Doppler:

$$
f_D T_s = \frac{f_D}{R_s}
$$

Where $R_s$ is the symbol rate. This parameter describes **how drastically the channel changes within each symbol period**.

| $f_D T_s$ | Scenario |
|-----------|----------|
| 0 | Stationary (block fading) |
| 0.001 | Low-speed mobility (walking) |
| 0.01 | Medium-speed mobility (urban driving) |
| 0.1 | High-speed mobility (high-speed rail, aircraft) |

### 4.3 Jakes Model and AR(1) Approximation

Classic Jakes model autocorrelation function:

$$
R_h(\Delta t) = J_0(2\pi f_D \Delta t)
$$

Where $J_0$ is the zeroth-order Bessel function.

For simulation simplicity, we use AR(1) approximation:

$$
h[n] = a \cdot h[n-1] + b \cdot w[n]
$$

Where:
- $a = J_0(2\pi f_D T_s)$: Correlation coefficient between adjacent symbols
- $b = \sqrt{1 - a^2}$: Innovation coefficient
- $w[n] \sim \mathcal{CN}(0, 1)$: Driving noise

### 4.4 Critical Impact on Alamouti

**Core Problem**: The Alamouti decoding formula assumes $h_1, h_2$ are constant during two time slots.

When the channel is time-varying, let the channel in the first time slot be $(h_1, h_2)$, and it becomes $(h_1', h_2')$ in the second time slot:

$$
y_1 = \frac{1}{\sqrt{2}}(h_1 s_1 + h_2 s_2) + n_1
$$

$$
y_2 = \frac{1}{\sqrt{2}}(-h_1' s_2^* + h_2' s_1^*) + n_2
$$

Using $(h_1, h_2)$ for decoding produces **Inter-Symbol Interference (ISI)**:

$$
z_1 = \alpha s_1 + \underbrace{\frac{h_2(h_2' - h_2)^*}{2\sqrt{2}} s_2^*}_{\text{ISI}} + \tilde{n}_1
$$

### 4.5 BER Floor Phenomenon

As $f_D T_s$ increases:
1. Channel estimation (based on frame header pilots) becomes outdated by frame end
2. Alamouti orthogonality is destroyed
3. BER approaches a **floor** that cannot be improved by increasing SNR

Simulation results:

| $f_D T_s$ | BER at SNR=20dB |
|-----------|-----------------|
| 0.000 | 6×10⁻³ |
| 0.002 | 4.6×10⁻² |
| 0.005 | 1.6×10⁻¹ |
| 0.010 | 3.1×10⁻¹ |
| 0.020 | 4.2×10⁻¹ (approaching random guess 0.5) |

### 4.6 Mitigation Method: Segmented Pilot Tracking

The `doppler_tracking=true` option in the code:
- Inserts pilots every `doppler_data_seg_len` data symbols
- Re-estimates the channel for each segment
- Trade-off: Reduced spectral efficiency (increased pilot overhead)

---

## 5. Code Parameter Description

### 5.1 Run Modes

| Parameter | Default | Description |
|-----------|---------|-------------|
| `fast_mode` | `false` | `true`: 120 frames, 4dB steps (fast); `false`: 500 frames, 2dB steps (accurate) |
| `run_rice_sweep` | `true` | Whether to run Rice K sweep |
| `run_corr_sweep` | `true` | Whether to run correlation sweep |
| `run_doppler_sweep` | `true` | Whether to run Doppler sweep |
| `doppler_tracking` | `false` | Whether to enable segmented pilot tracking |

### 5.2 Channel Parameters

| Parameter | Description | Sweep Range |
|-----------|-------------|-------------|
| `K` (dB) | Rice factor | [0, 5, 10] dB |
| `rho` | Spatial correlation coefficient | [0, 0.5, 0.9] |
| `fdTs` | Normalized Doppler | [0, 0.002, 0.005, 0.01, 0.02] |

### 5.3 System Parameters (Consistent with Main Project)

- Modulation: 16-QAM (unnormalized, constellation points {±1, ±3})
- Data symbols: 1000/frame
- Pilots: 10 symbols/antenna (orthogonal pilots)
- Pulse shaping: RRC, rolloff=0.5, span=16, sps=2

---

## 6. Usage Example

```matlab
% Accurate mode, only run Doppler sweep
fast_mode = false;
run_rice_sweep = false;
run_corr_sweep = false;
run_doppler_sweep = true;
doppler_tracking = false;

Main_Alamouti_Extensions;
```

---

## 7. Key Points for Result Interpretation

### 7.1 Rice K Sweep
- Higher K → Lower BER (LOS provides stability)
- Improvement is limited since Alamouti already provides diversity

### 7.2 Spatial Correlation Sweep
- Higher ρ → Higher BER (diversity degradation)
- Performance significantly degrades at ρ=0.9

### 7.3 Doppler Sweep
- Noticeable BER floor appears when fdTs > 0.01
- BER cannot be reduced even at very high SNR
- This is a **fundamental limitation** of Alamouti in high-mobility scenarios

---

## 8. References

1. S. M. Alamouti, "A simple transmit diversity technique for wireless communications," IEEE JSAC, 1998.
2. W. C. Jakes, *Microwave Mobile Communications*, Wiley, 1974.
3. D. Tse and P. Viswanath, *Fundamentals of Wireless Communication*, Cambridge, 2005.

# convolution_RX.m: Line-by-Line Code Explanation

This document provides a **comprehensive, line-by-line explanation** of every code statement in `convolution_RX.m`, which implements the receiver-side matched filter and downsampling operation.

---

## Function Overview

**Purpose**: Apply matched filter (Root Raised Cosine) to the received signal and downsample to symbol rate.

**Inputs**:
- `signal`: Received signal at sample rate (after channel and AWGN)
- `g`: Filter impulse response (RRC filter)
- `sps`: Samples per symbol (oversampling factor)

**Output**:
- `symb_RX`: Symbol-rate received symbols (column vector)

---

## Line-by-Line Explanation

### Lines 1-17: Function Header and Documentation

```matlab
function symb_RX = convolution_RX( signal , g , sps )
%% CONVOLUTION_RX Convolution (and downsampling)
% This function implements a disrete convolution between a waveform and the 
% impulse response of the RX filter. The resulting vector is then cropped and 
% downsampled in order to retrieve the RX symbol vector.
% 
% *Inputs:*
%% 
% * _signal_: signal vector of size _[ ( nb_symb - 1 ) * sps + length( g ) ] 
% x 1_
% * _g_: Impulse Response (IR) of the pulse of size _1 x_ length( _g_ )
% * _sps_: number of samples per symbol 
%% 
% *Ouput:*
%% 
% * _symb_RX_: filtered signal which corresponds to received symbols (column 
% vector)
```

**Explanation:**
- **Line 1**: Function declaration. Returns `symb_RX` (column vector of received symbols).
- **Lines 2-17**: Documentation block explaining:
  - Purpose: Discrete convolution with RX filter, then downsampling.
  - Input `signal`: Sample-rate waveform (includes filter delay).
  - Input `g`: Filter impulse response (RRC).
  - Input `sps`: Oversampling factor (e.g., 2).
  - Output: Symbol-rate received symbols.

### Lines 18-34: Discrete Convolution

```matlab
%% Discrete Convolution
% This part is simliar to the "Discrete Convolution" section of the function 
% "convolution_TX" and can therefore be directly copy-pasted (only variable names 
% need to be adjusted)
% Code the discrete convolution here
Lg = length(g);
L = (Lg-1)/2;
nb_sample = length(signal);
UPsymb_RX_filled = [zeros(2*L,1); signal; zeros(2*L,1)];
conv_sig_RX = zeros(nb_sample+4*L,1);
for i=L+1:nb_sample+3*L
    tp_sum = 0;
    for j=-L:L
        tp_sum = tp_sum + UPsymb_RX_filled(i-j)*g(j+L+1);
    end
    conv_sig_RX(i) = tp_sum;
end
```

**Explanation:**
- **Line 18**: Section header for discrete convolution.
- **Lines 19-21**: Comment noting similarity to `convolution_TX` (same convolution algorithm).
- **Line 23**: `Lg = length(g)`: Get filter length (e.g., 33 for span=16, sps=2).
- **Line 24**: `L = (Lg-1)/2`: Calculate half-length for indexing.
  - Example: `Lg = 33` → `L = 16`.
  - Filter is symmetric around index `L+1`.
- **Line 25**: `nb_sample = length(signal)`: Get number of input samples.
- **Line 26**: `UPsymb_RX_filled = [zeros(2*L,1); signal; zeros(2*L,1)]`: Zero-padding.
  - **Why**: Convolution needs padding to handle boundary effects.
  - Structure: `[2L zeros | signal | 2L zeros]`.
  - Total length: `nb_sample + 4*L`.
- **Line 27**: `conv_sig_RX = zeros(nb_sample+4*L,1)`: Initialize output vector.
- **Line 28**: `for i=L+1:nb_sample+3*L`: Loop over valid output indices.
  - Start at `L+1` (skip padding region).
  - End at `nb_sample+3*L` (before trailing padding).
- **Line 29**: `tp_sum = 0`: Initialize accumulator for convolution sum.
- **Line 30**: `for j=-L:L`: Loop over filter taps.
  - `j` ranges from `-L` to `+L` (filter span).
- **Line 31**: `tp_sum = tp_sum + UPsymb_RX_filled(i-j)*g(j+L+1)`: Convolution sum.
  - `UPsymb_RX_filled(i-j)`: Input sample at position `i-j`.
  - `g(j+L+1)`: Filter tap (shifted index: `j=-L` → `g(1)`, `j=0` → `g(L+1)`, `j=L` → `g(2L+1)`).
  - This implements: `y[n] = Σ g[k]·x[n-k]`.
- **Line 33**: `conv_sig_RX(i) = tp_sum`: Store convolution result.

**Mathematical Formulation:**
The discrete convolution is:
```
y[n] = Σ_{k=-L}^{L} g[k+L+1] · x[n-k]
```
where `x` is the zero-padded input signal and `g` is the filter impulse response.

### Lines 35-40: Vector Cropping

```matlab
% Vector cropping
% This part is simliar to the "Discrete Convolution" section of the function 
% "convolution_TX" and can therefore be directly copy-pasted (only variable names 
% need to be adjusted).
% Crop the resulting vector here
UPsymb_RX = conv_sig_RX(L+1:nb_sample+3*L);
```

**Explanation:**
- **Lines 35-38**: Comments noting similarity to TX function.
- **Line 40**: `UPsymb_RX = conv_sig_RX(L+1:nb_sample+3*L)`: Crop to remove padding effects.
  - Removes first `L` samples (filter transient).
  - Keeps `nb_sample+2*L` samples (valid convolution output).
  - This is the filtered signal at **sample rate** (still oversampled).

### Lines 41-50: Downsampling

```matlab
%% Downsampling
% In order to retrieve the symbols at RX, the received filtered signal needs 
% to be sampled at the sampling rate (every _pT_ in slides 21-24 of the course 
% 2). This corresponds to downsample the filtered signal by a factor _sps_.
% Code the downsampling here
nb_symb = floor((nb_sample+2*L-1)/sps) +1;
symb_RX = zeros(nb_symb,1);
for i=1:nb_symb
    symb_RX(i) = UPsymb_RX((i-1)*sps + 1);
end
```

**Explanation:**
- **Line 41**: Section header for downsampling.
- **Lines 42-45**: Comments explaining downsampling purpose:
  - Filtered signal is at sample rate (oversampled).
  - Need to extract symbols at symbol rate (every `sps` samples).
- **Line 46**: `nb_symb = floor((nb_sample+2*L-1)/sps) +1`: Calculate number of symbols.
  - `nb_sample+2*L`: Length of cropped filtered signal.
  - `floor((nb_sample+2*L-1)/sps)`: Number of complete symbol intervals.
  - `+1`: Add one for the last symbol.
  - Example: `nb_sample+2*L = 2040`, `sps = 2` → `nb_symb = 1020`.
- **Line 47**: `symb_RX = zeros(nb_symb,1)`: Initialize symbol vector.
- **Line 48**: `for i=1:nb_symb`: Loop over symbols.
- **Line 49**: `symb_RX(i) = UPsymb_RX((i-1)*sps + 1)`: Extract symbol at optimal sampling instant.
  - `(i-1)*sps + 1`: Sample index for symbol `i`.
    - Symbol 1: sample 1.
    - Symbol 2: sample 3 (`sps=2`).
    - Symbol 3: sample 5.
    - etc.
  - This implements **symbol-rate sampling** (downsampling by factor `sps`).

**Why Sample at `(i-1)*sps + 1`?**
- The matched filter output has its peak at the center of each symbol interval.
- For `sps=2`, samples are at indices 1, 3, 5, ... (every 2 samples).
- This corresponds to the optimal sampling instants (Nyquist criterion).

### Line 51: End of Function

```matlab
end
```

**Explanation:**
- Marks the end of the function.

---

## Summary

This function performs two main operations:

1. **Matched Filtering (Convolution)**: 
   - Convolves received signal with RRC filter impulse response.
   - Implements matched filter for optimal SNR (maximizes signal-to-noise ratio).
   - Uses zero-padding to handle boundaries.

2. **Downsampling**:
   - Extracts symbols at symbol rate from oversampled filtered signal.
   - Samples at optimal instants: `(i-1)*sps + 1` for symbol `i`.
   - Reduces data rate from sample rate to symbol rate.

**Key Points**:
- The matched filter (RRC) is the optimal receiver filter for AWGN channels.
- When combined with the TX RRC filter, the cascade forms a full Raised Cosine filter (zero ISI at symbol instants).
- Downsampling extracts the symbol-rate decision variables for subsequent detection.

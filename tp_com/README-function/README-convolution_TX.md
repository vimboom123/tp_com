# convolution_TX.m: Line-by-Line Code Explanation

This document provides a **comprehensive, line-by-line explanation** of every code statement in `convolution_TX.m`, which implements the transmitter-side pulse shaping and upsampling operation.

---

## Function Overview

**Purpose**: Upsample symbol stream and apply pulse shaping filter (Root Raised Cosine) for transmission.

**Inputs**:
- `symb_TX`: Symbol-rate complex symbols (column vector)
- `g`: Filter impulse response (RRC filter)
- `sps`: Samples per symbol (oversampling factor)

**Output**:
- `signal`: Sample-rate pulse-shaped signal (column vector)

---

## Line-by-Line Explanation

### Lines 1-18: Function Header and Documentation

```matlab
function signal = convolution_TX( symb_TX , g , sps )
%% CONVOLUTION_TX Convolution (and upsampling)
% This function implements a disrete convolution between a symbol stream and 
% the impulse response of the TX filter (i.e., the pulse shape). To do so, the 
% symbol stream is first upsampled in order to match the sampling rate defined 
% by the pulse IR, and then the discrete convolution is performed).
% 
% *Inputs:*
%% 
% * _symb_TX_: complex symbol stream vector of size _nb_symb x 1_ where _nb_symb 
% =_ length( _symb_TX_ )
% * _g_: Impulse Response (IR) of the pulse of size _1 x_ length( _g_ )
% * _sps_: number of samples per symbol 
%% 
% *Ouput:*
%% 
% * _signal_: result of the convolution, vector of size _[ ( nb_symb - 1 ) * 
% sps + length( g ) ] x 1_
```

**Explanation:**
- **Line 1**: Function declaration. Returns `signal` (sample-rate pulse-shaped waveform).
- **Lines 2-18**: Documentation explaining:
  - Purpose: Upsample symbols, then apply pulse shaping filter.
  - Input `symb_TX`: Symbol-rate complex symbols.
  - Input `g`: Filter impulse response (RRC).
  - Input `sps`: Oversampling factor.
  - Output: Sample-rate signal (includes filter delay).

### Lines 19-31: Upsampling

```matlab
%% Upsampling
% In order to match the sampling rate used to design the filter impulse response 
% _g_, the symbol vector _symb_TX_ must first be upsampled as illustrated below 
% (zero-padding):
% 
% 
% Code the upsampling here
nb_symb = length( symb_TX );
nb_sample = nb_symb*sps - sps + 1;
UPsymb_TX = zeros(nb_sample,1);
for i=1:nb_symb
    UPsymb_TX( (i-1)*sps + 1 ) = symb_TX(i);
end
```

**Explanation:**
- **Line 19**: Section header for upsampling.
- **Lines 20-24**: Comments explaining upsampling:
  - Filter `g` is designed at sample rate.
  - Symbols must be upsampled to match sample rate.
  - Upsampling = zero-padding (insert zeros between symbols).
- **Line 26**: `nb_symb = length(symb_TX)`: Get number of input symbols.
- **Line 27**: `nb_sample = nb_symb*sps - sps + 1`: Calculate upsampled length.
  - Formula: `(nb_symb-1)*sps + 1`.
  - Example: `nb_symb=1000`, `sps=2` → `nb_sample = 1999`.
  - **Why `-sps + 1`?**: Last symbol doesn't need trailing zeros.
- **Line 28**: `UPsymb_TX = zeros(nb_sample,1)`: Initialize upsampled vector (all zeros).
- **Line 29**: `for i=1:nb_symb`: Loop over symbols.
- **Line 30**: `UPsymb_TX((i-1)*sps + 1) = symb_TX(i)`: Place symbol at correct position.
  - Symbol 1: position 1.
  - Symbol 2: position 3 (`sps=2`).
  - Symbol 3: position 5.
  - etc.
  - All other positions remain zero (zero-padding).

**Upsampling Example** (`sps=2`):
```
Input symbols:  [s1, s2, s3, ...]
Upsampled:      [s1, 0, s2, 0, s3, 0, ...]
                ↑   ↑   ↑   ↑   ↑   ↑
                1   2   3   4   5   6
```

### Lines 32-74: Discrete Convolution

```matlab
%% Discrete Convolution
% The discrete convolution between the upsampled symbol stream $ s_{up}$ and 
% the filter Impulse Response (IR) $g$ is defined as $\left( s_{up} * g \right)[n]=\sum_{l=-\infty}^{+\infty}s_{up}[n-l]g[l]$. 
% In our case, $g$ represents the filter Impulse Response (IR) which is finite 
% (FIR filter). So the convolution can be written as a finite summation: 
% 
% $$\left( s_{up} * g \right)[n]=\sum_{l=-L}^{L}s_{up}[n-l]g[l]$$
% 
% with $n \in [1 : nb_{sample}]$ and $2L+1$ is the number of samples of the 
% filter IR.
% 
% To implement this finite summation in Matlab, a special care to vector indexes 
% should be observed. Indeed, Matlab does not handle negative indexes. So the 
% summation should be coded as:
% 
% $$\left( s_{up} * g \right)[n]=\sum_{l=-L}^{L}s_{up}[n-l]g[l+L+1]$$
% 
% An issue also raises when:
%% 
% * $n\leq l$: indexes become negative in $ s_{up}$
% * $n - l>nb_{sample}$: indexes exceed the number of elements in $ s_{up}$
%% 
% To avoid that, it is possible to fill the $ s_{up}$ vector with $2L$ zeros 
% at the begining and $2L$ zeros at the end such as:
% 
% $ s^{filled}_{up}=[ 0~\cdots~0~s_{up}~0~\cdots~0]$. The convolution to implement 
% becomes then:
% 
% $$\left(  s^{filled}_{up} * g \right)[n]=\sum_{l=-L}^{L} s^{filled}_{up}[n-l]g[l+L+1]$$
% 
% with $n \in [L+1 : nb_{sample}+3L]$.
% Code the discrete convolution here
Lg = length(g);
L = (Lg-1)/2;
UPsymb_TX_filled = [zeros(2*L,1); UPsymb_TX; zeros(2*L,1)];
conv_sig_TX = zeros(nb_sample+4*L,1);
for i=L+1:nb_sample+3*L
    tp_sum = 0;
    for j=-L:L
        tp_sum = tp_sum + UPsymb_TX_filled(i-j)*g(j+L+1);
    end
    conv_sig_TX(i) = tp_sum;
end
```

**Explanation:**
- **Lines 32-62**: Detailed mathematical documentation:
  - Convolution definition: `y[n] = Σ s[n-l]·g[l]`.
  - Finite summation: `y[n] = Σ_{l=-L}^{L} s[n-l]·g[l]`.
  - Index shifting: `g[l+L+1]` to handle negative indices.
  - Zero-padding solution for boundary handling.
- **Line 64**: `Lg = length(g)`: Get filter length (e.g., 33 for span=16, sps=2).
- **Line 65**: `L = (Lg-1)/2`: Calculate half-length.
  - Example: `Lg = 33` → `L = 16`.
  - Filter is symmetric around index `L+1`.
- **Line 66**: `UPsymb_TX_filled = [zeros(2*L,1); UPsymb_TX; zeros(2*L,1)]`: Zero-padding.
  - Structure: `[2L zeros | upsampled symbols | 2L zeros]`.
  - **Why**: Prevents index errors at boundaries during convolution.
  - Total length: `nb_sample + 4*L`.
- **Line 67**: `conv_sig_TX = zeros(nb_sample+4*L,1)`: Initialize output vector.
- **Line 68**: `for i=L+1:nb_sample+3*L`: Loop over valid output indices.
  - Start at `L+1` (skip leading padding).
  - End at `nb_sample+3*L` (before trailing padding).
- **Line 69**: `tp_sum = 0`: Initialize accumulator.
- **Line 70**: `for j=-L:L`: Loop over filter taps.
  - `j` ranges from `-L` to `+L`.
- **Line 71**: `tp_sum = tp_sum + UPsymb_TX_filled(i-j)*g(j+L+1)`: Convolution sum.
  - `UPsymb_TX_filled(i-j)`: Input sample at position `i-j`.
  - `g(j+L+1)`: Filter tap (index shift: `j=-L` → `g(1)`, `j=0` → `g(L+1)`, `j=L` → `g(2L+1)`).
  - Implements: `y[n] = Σ g[k]·x[n-k]`.
- **Line 73**: `conv_sig_TX(i) = tp_sum`: Store result.

**Mathematical Formulation:**
```
y[n] = Σ_{k=-L}^{L} g[k+L+1] · x[n-k]
```
where `x` is the zero-padded upsampled symbol stream.

### Lines 75-83: Vector Cropping

```matlab
% Vector cropping
% The vector resulting from the convolution of the filter IR and the symbol 
% stream contains zeros at its begining and its end. They should be removed to 
% obtain the appropriate vector size of _[ ( nb_symb - 1 ) * sps + length( g ) 
% ] x 1_.
% Crop the resulting vector here
% tail = (nb_sample+2*L) - (nb_sample+Lg-1);
% signal = conv_sig_TX(tail/2 +1:end-tail/2);
signal = conv_sig_TX(L+1:nb_sample+3*L);
```

**Explanation:**
- **Lines 75-79**: Comments explaining cropping:
  - Convolution output has leading/trailing zeros from padding.
  - Need to remove them to get correct size.
  - Final size: `(nb_symb-1)*sps + length(g)`.
- **Lines 81-82**: Commented alternative cropping method (not used).
- **Line 83**: `signal = conv_sig_TX(L+1:nb_sample+3*L)`: Crop to remove padding.
  - Removes first `L` samples (leading padding).
  - Keeps `nb_sample+2*L` samples (valid convolution + filter tail).
  - This is the **final pulse-shaped signal** at sample rate.

**Why Keep `nb_sample+2*L`?**
- The filter has a "tail" (non-zero response after the last symbol).
- This tail is needed for proper matched filtering at the receiver.
- Total length: `(nb_symb-1)*sps + Lg = nb_sample + 2*L`.

### Lines 84-88: End Comments

```matlab
%% 
% The size of the resulting signal, is equal to the upsampled stream PLUS the 
% filter length - 1. There is therefore a tail at the begining and the end of 
% the time domain vector introduced by the filter delay.
end
```

**Explanation:**
- **Lines 85-87**: Final comments explaining output size:
  - Signal length = upsampled length + filter length - 1.
  - Filter introduces delay (leading tail) and extends beyond last symbol (trailing tail).
- **Line 88**: End of function.

---

## Summary

This function performs two main operations:

1. **Upsampling**:
   - Converts symbol-rate input to sample rate.
   - Inserts zeros between symbols (zero-padding).
   - Example: `sps=2` → each symbol followed by one zero.

2. **Pulse Shaping (Convolution)**:
   - Applies RRC filter to upsampled symbols.
   - Implements discrete convolution with zero-padding for boundaries.
   - Produces smooth, bandlimited waveform suitable for transmission.

**Key Points**:
- The RRC filter shapes the spectrum to reduce out-of-band emissions.
- When combined with RX RRC filter, the cascade forms a full Raised Cosine filter (zero ISI).
- The output signal is ready for transmission (after channel and AWGN).

**Relationship to RX Function**:
- TX: `symbols → upsampling → pulse shaping → sample-rate signal`.
- RX: `sample-rate signal → matched filter → downsampling → symbols`.
- The RX matched filter is the same RRC filter (optimal for AWGN channels).

# raised_cosine.m: Line-by-Line Code Explanation

This document provides a **comprehensive, line-by-line explanation** of every code statement in `raised_cosine.m`, which designs Raised Cosine (RC) and Root Raised Cosine (RRC) filter impulse responses.

---

## Function Overview

**Purpose**: Generate impulse response of Raised Cosine or Root Raised Cosine filter for pulse shaping.

**Inputs**:
- `rolloff`: Rolloff factor β (0 ≤ β ≤ 1)
- `span`: Filter span in symbols (e.g., 16)
- `sps`: Samples per symbol (e.g., 2)
- `shape`: Filter type (`'normal'` for RC, `'sqrt'` for RRC)

**Output**:
- `g`: Filter impulse response (row vector of length `span*sps + 1`)

---

## Line-by-Line Explanation

### Lines 1-23: Function Header and Validation

```matlab
function g = raised_cosine( rolloff , span , sps , shape )
%% RAISED_COSINE (Root) Raised Cosine Filter
% This function designs the Impulse Response (i.e., discrete-time pulse shape) 
% of a Raised Cosine (RC) filter and Root Raised Cosine (RRC) filter, typically 
% used for pulse shaping.
% 
% *Inputs:*
%% 
% * _rolloff_: rolloff factor of the raised cosine filter ($0\le \text{rolloff}\le 
% 1  $)
% * _span_: pulse impulse response duration in terms of symbols
% * _sps_: number of samples per symbol
% * _shape_: select Raised Cosine filter (_shape_ = 'normal') or Root Raised 
% Cosine filter (_shape_ = 'sqrt')
%% 
% *Ouput:*
%% 
% * _g_: Impulse Response (IR) of the pulse of size _1 x ( span . sps + 1 )_
    nb_sample_per_pulse = span * sps ;
    if rem( nb_sample_per_pulse , 2) ~= 0
        error('The product of sps and span must be even')
    end
    eps = 1e-3;
```

**Explanation:**
- **Line 1**: Function declaration. Returns `g` (filter impulse response).
- **Lines 2-18**: Documentation explaining inputs and output.
- **Line 19**: `nb_sample_per_pulse = span * sps`: Calculate samples per pulse.
  - Example: `span=16`, `sps=2` → `32` samples.
- **Lines 20-22**: Validation: `span*sps` must be even.
  - `rem(a, 2)`: Remainder when divided by 2.
  - If odd, throw error (needed for symmetric filter design).
- **Line 23**: `eps = 1e-3`: Tolerance for checking special cases (indeterminate forms).

### Lines 24-98: Theoretical Background

```matlab
%% Guidelines for the pulse IR design
% General expressions
% The pulse shape of a *raised cosine* spectrum is given by the following relationship:
% 
% $$g_{RC}\left(t\right)=\text{sinc}\left(\pi t / T\right)\frac{\cos\left( \pi 
% \beta t / T\right)}{1-4 \beta^2 t^2 / T^2}$$
% 
% The pulse shape of a *root raised cosine* spectrum is given by the following 
% relationship:
% 
% $$g_{RRC}\left(t\right)=\frac{ 4\beta\cos\left[ \left( 1+\beta \right)\pi 
% t / T  \right]+\frac{\sin\left[ \left( 1-\beta \right)\pi t / T  \right]} {\left( 
% t/T\right)}}{ \pi*\sqrt{T} * \left[ 1 - 16 \beta^2t^2/T^2\right]}$$ 
% 
% where $\beta$ is the roll-off factor and $T$ is the symbol duration.
```

**Explanation:**
- **Lines 24-38**: Mathematical formulas for RC and RRC filters.
- **RC formula**: `g_RC(t) = sinc(πt/T) · cos(πβt/T) / (1-4β²t²/T²)`.
- **RRC formula**: More complex expression involving cosine and sine terms.

### Lines 99-126: Raised Cosine (RC) Filter Implementation

```matlab
    if strncmp(shape, 'normal', 1)
    % Code here the raised cosine (RC) impulse response (output: g)
    beta = rolloff;
    t_T = -span/2 : 1/sps : +span/2;
    g = zeros(1,span*sps+1);
    for i=1:span*sps+1
        if ( abs(abs(t_T(i)) - 1/(2*beta)) <= eps )
            g(i) = sinc(1/(2*beta))*pi/4;
        else
            g(i) = sinc(t_T(i))*cos(pi*beta*t_T(i))/(1-4*t_T(i)^2 * beta^2);
        end
    end
```

**Explanation:**
- **Line 115**: `if strncmp(shape, 'normal', 1)`: Check if RC filter (case-insensitive).
- **Line 117**: `beta = rolloff`: Rolloff factor (0 to 1).
- **Line 118**: `t_T = -span/2 : 1/sps : +span/2`: Normalized time vector.
  - Range: `[-span/2, +span/2]` in steps of `1/sps`.
  - Example: `span=16`, `sps=2` → `[-8, -7.5, -7, ..., +7.5, +8]`.
  - Length: `span*sps + 1` samples.
- **Line 119**: `g = zeros(1,span*sps+1)`: Initialize filter vector.
- **Line 120**: `for i=1:span*sps+1`: Loop over all time samples.
- **Lines 121-123**: Handle indeterminate form at `|t/T| = 1/(2β)`.
  - **Line 121**: Check if `|t/T| ≈ 1/(2β)` (within tolerance `eps`).
  - **Line 122**: Use L'Hôpital's rule limit: `g = sinc(1/(2β))·π/4`.
- **Lines 124-125**: Normal case: Apply RC formula.
  - `sinc(t_T(i))`: `sinc(πt/T)` (MATLAB's `sinc` includes π).
  - `cos(pi*beta*t_T(i))`: `cos(πβt/T)`.
  - `1-4*t_T(i)^2 * beta^2`: Denominator `1-4β²t²/T²`.

### Lines 127-162: Root Raised Cosine (RRC) Filter Implementation

```matlab
    elseif strncmp(shape, 'sqrt', 1)
    % Code here the root raised cosine (RRC) impulse response (output: g)
    beta = rolloff;
    t_T = -span/2 : 1/sps : +span/2;
    g = zeros(1,span*sps+1);
    for i=1:span*sps+1
        if ( abs(abs(t_T(i)) - 0) <= eps )
            g(i) = (4*beta +  (1-beta)*pi) / pi;
        
        elseif ( abs(abs(t_T(i)) - 1/(4*beta)) <= eps )
            g(i) = (1+beta)*sin((1+beta)*pi/(4*beta))/2 - (1-beta)*cos((1-beta)*pi/(4*beta))/2 + 2*beta*sin((1-beta)*pi/(4*beta))/pi;
        
        else
            g(i) = (4*beta*cos((1+beta)*pi*t_T(i)) + sin((1-beta)*pi*t_T(i))/t_T(i)) / (pi*(1-16*beta^2 *t_T(i)^2));
        end
    end
```

**Explanation:**
- **Line 147**: `elseif strncmp(shape, 'sqrt', 1)`: Check if RRC filter.
- **Line 149**: `beta = rolloff`: Rolloff factor.
- **Line 150**: `t_T = -span/2 : 1/sps : +span/2`: Normalized time vector (same as RC).
- **Line 151**: `g = zeros(1,span*sps+1)`: Initialize filter vector.
- **Line 152**: `for i=1:span*sps+1`: Loop over all time samples.
- **Lines 153-155**: Handle indeterminate form at `t/T = 0`.
  - **Line 153**: Check if `|t/T| ≈ 0` (within tolerance).
  - **Line 154**: Use limit: `g = (4β + (1-β)π) / π`.
- **Lines 156-158**: Handle indeterminate form at `|t/T| = 1/(4β)`.
  - **Line 156**: Check if `|t/T| ≈ 1/(4β)`.
  - **Line 157**: Use L'Hôpital's rule limit (complex expression).
- **Lines 159-161**: Normal case: Apply RRC formula.
  - Numerator: `4β·cos((1+β)πt/T) + sin((1-β)πt/T)/(t/T)`.
  - Denominator: `π(1-16β²t²/T²)`.

### Lines 164-171: Filter Energy Normalization

```matlab
    end
%% Filter Energy Normalization 
% The filter amplitude is normalized in order to exhibit a unit energy. With 
% this normalization, the amplitude of a TX symbol will remain constant after 
% passing successively through TX and RX filter (if the propagation channel is 
% not considered, i.e., $h=1$).
    g = g / sqrt( sum( g.^2 ) ) ;                                   
end
```

**Explanation:**
- **Line 164**: End of `if-elseif` block.
- **Lines 165-169**: Comments explaining normalization:
  - Normalize to unit energy.
  - Ensures symbol amplitude preserved through TX+RX filters.
- **Line 170**: `g = g / sqrt(sum(g.^2))`: Normalize to unit energy.
  - `g.^2`: Element-wise square (power).
  - `sum(...)`: Total energy.
  - `sqrt(...)`: RMS value.
  - Division: Normalize so `Σ|g|² = 1`.

---

## Summary

This function generates pulse shaping filters:

1. **Raised Cosine (RC)**:
   - Formula: `g_RC(t) = sinc(πt/T)·cos(πβt/T) / (1-4β²t²/T²)`.
   - Handles indeterminate form at `|t/T| = 1/(2β)`.

2. **Root Raised Cosine (RRC)**:
   - More complex formula with cosine and sine terms.
   - Handles indeterminate forms at `t/T = 0` and `|t/T| = 1/(4β)`.

**Key Points**:
- Both filters are normalized to unit energy.
- RRC is used at both TX and RX; their cascade forms a full RC filter (zero ISI).
- The filter is truncated to `span` symbols (FIR approximation of IIR filter).
- Special cases (indeterminate forms) are handled using analytical limits.

**Relationship to Other Functions**:
- **Output**: Used in `convolution_TX.m` and `convolution_RX.m` for pulse shaping.
- **Purpose**: Bandlimit the signal and enable zero-ISI transmission.

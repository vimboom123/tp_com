# symbol_estimation_QAM.m: Line-by-Line Code Explanation

This document provides a **comprehensive, line-by-line explanation** of every code statement in `symbol_estimation_QAM.m`, which implements minimum-distance (nearest-neighbor) detection for QAM symbols.

---

## Function Overview

**Purpose**: Perform optimal symbol detection using minimum Euclidean distance (nearest-neighbor detector) for AWGN channels.

**Inputs**:
- `symb_RX`: Noisy received symbols (column vector)
- `nb_bit_per_symb`: Bits per symbol (2 for QPSK, 4 for 16-QAM)
- `nb_symb`: Number of symbols

**Output**:
- `symb_RX_estimated`: Hard-decided symbols (nearest constellation points)

---

## Line-by-Line Explanation

### Lines 1-17: Function Header and Initialization

```matlab
function symb_RX_estimated = symbol_estimation_QAM( symb_RX , nb_bit_per_symb , nb_symb )
%% SYMBOL_ESTIMATION_QAM Optimal Detection for the Vector AWGN Channel (minimum-distance detector)
% This function estimates noisy complex symbol values using the minimum Euclidean 
% distance as an estimator with respect to some QAM constellations.
% 
% *Inputs:*
%% 
% * _symb_RX_: noisy RX symbol vector of size _nb_symb x 1_ where _nb_symb = 
% nb_bit / nb_bit_per_symb_
% * _nb_bit_per_symb_: number of bits per symbol
% * _nb_symb_: number of symbols in the  symbol stream (= length of _symb_RX_)
%% 
% *Ouputs:*
%% 
% * _symb_RX_estimated_: estimated complex symbol stream vector of size _nb_symb 
% x 1_ where _nb_symb = nb_bit / nb_bit_per_symb_
symb_RX_estimated = zeros( nb_symb , 1 ) ;
```

**Explanation:**
- **Line 1**: Function declaration. Returns `symb_RX_estimated` (column vector of hard-decided symbols).
- **Lines 2-16**: Documentation explaining:
  - Purpose: Minimum-distance detection (optimal for AWGN).
  - Inputs: Noisy symbols, bits per symbol, number of symbols.
  - Output: Hard-decided symbols.
- **Line 17**: `symb_RX_estimated = zeros(nb_symb, 1)`: Initialize output vector.

### Lines 18-72: Theoretical Background

```matlab
%% Minimum-distance detector: theory
% This function implements an optimal detection in the case of a transmission 
% over an additive white Gaussian noise (AWGN) channel whose vector representation 
% can be written as:
% 
% $\mathbf{r}=\mathbf{s_m}+\mathbf{n}$,          $1\leq m \leq M$
% 
% where $\mathbf{r}$ is the received symbol, $\mathbf{s_m}$ is the transmitted 
% symbol among $M$possible symbols within a given constellation, and $\mathbf{n}$ 
% is a white Gaussiance noise (i.e., its real and imaginary components are i.i.d. 
% zero-mean Gaussian random variables with variance $\frac{N_0}{2}$). The vectors 
% considered here are 2-dimensional vectors from a 2D orthonormal plane, and can 
% therefore be conveniently represented in a complex plane (i.e., 2D symbol space 
% <=>  IQ plane: $\mathbf{s_m}=\{ I_m, Q_m \}$).
% 
% The role of the estimator at the receiver is to decide which symbol has been 
% sent based on the observation of $\mathbf{r}$, so the optimal decision rule 
% can be written as:
% 
% $\hat m = \arg\max_{1\leq m \leq M} P_mp (\mathbf{r} \mid\mathbf{s_m})$, where 
% $P_m$ is the _a priori_ probability of a given symbol $\mathbf{s_m}$ being transmitted 
% and $p (\mathbf{r} \mid\mathbf{s_m})$ is the conditional probability density 
% function of the received symbol $\mathbf{r}$ knowing $\mathbf{s_m}$ has been 
% transmitted. This is known as the *maximum a posteriori probability (MAP)* rule. 
% It consists in selecting the symbol index $m$ within a _M_-size that by maximizes 
% the probability $P_mp (\mathbf{r} \mid\mathbf{s_m})$.
% 
% In the case of equiprobable symbols (i.e., there is the same probability for 
% all symbols within a constellation to be transmitted), $P_m = \frac{1}{M}$ for 
% all  $1\leq m \leq M$. The optimal detection rule reduces then to: $\hat m = 
% \arg\max_{1\leq m \leq M} p (\mathbf{r} \mid\mathbf{s_m})$. This is known as 
% the *maximum likelihood estimator (MLE)* _(the MLE is not optimal unless symbols 
% are equiprobable in their occurence)._
% 
% In the scenario of a vector AWGN channel and assuming equiprobability between 
% symbols, both MAP and MLE can be implemented by a *minimum-distance (or nearest-neighbor) 
% detector*:
% 
% $\hat m = \arg\min_{1\leq m \leq M} \mid\mid \mathbf{r} - \mathbf{s_m} \mid\mid 
% $.
% 
% For more details regarding optimal receiver, see chapter 4 from _"Digital 
% Communications", 5th Edition, J.G Proakis and M. Salehi, McGraw Hill_
%% Guidelines: what to do?
% 
% 
% The receiver receives $\mathbf{r}$ and looks among all $\mathbf{s_m}$ to find 
% the one that is closest to $\mathbf{r}$ using standard Euclidean distance. This 
% needs to be done for QPSK and 16QAM mapping.
% 
% In the above example, if the input symbol is _symb_RX_ = *r*, the ouput should 
% then be _symb_RX_estimated_ = 1 + 1i, since the m=1 symbol is the one that minimizes 
% the Euclidean distance between the received symbol and the symbols from the 
% original constellation.
```

**Explanation:**
- **Lines 18-60**: Detailed theoretical background:
  - **Channel model**: `r = s_m + n` (AWGN).
  - **MAP rule**: Maximize `P_m · p(r|s_m)`.
  - **MLE rule**: Maximize `p(r|s_m)` (for equiprobable symbols).
  - **Minimum-distance detector**: `m̂ = argmin ||r - s_m||` (optimal for AWGN).
- **Lines 61-72**: Implementation guidelines:
  - Find nearest constellation point using Euclidean distance.
  - Works for both QPSK and 16-QAM.

### Lines 74-88: QPSK (4-QAM) Detection

```matlab
%% 4QAM constellation (<=> QPSK)
% The QPSK constellation is as shown below:
% 
% 
if nb_bit_per_symb == 2
    
    % Code here the symbol estimation function for QPSK constellation (output: symb_RX_estimated)
    Lm = [-1+1i 1+1i 1-1i -1-1i];
    for k=1:nb_symb
        Ld = dist(symb_RX(k)*ones(1,4),Lm);
        [~,minD] = min(Ld);
        symb_RX_estimated(k) = Lm(minD);
    end
    
end
```

**Explanation:**
- **Line 74**: Section header for QPSK.
- **Line 78**: `if nb_bit_per_symb == 2`: Check if QPSK mode.
- **Line 80**: Comment: Symbol estimation for QPSK.
- **Line 81**: `Lm = [-1+1i 1+1i 1-1i -1-1i]`: QPSK constellation points.
  - All 4 possible symbols: `(-1±1i), (+1±1i)`.
  - Order: `[-1+1i, +1+1i, +1-1i, -1-1i]`.
- **Line 82**: `for k=1:nb_symb`: Loop over received symbols.
- **Line 83**: `Ld = dist(symb_RX(k)*ones(1,4),Lm)`: Calculate distances.
  - `symb_RX(k)*ones(1,4)`: Replicate received symbol 4 times.
  - `dist(..., Lm)`: Calculate Euclidean distance to each constellation point.
  - Result: `Ld` is a vector of 4 distances.
- **Line 84**: `[~,minD] = min(Ld)`: Find minimum distance index.
  - `min(Ld)`: Returns `[min_value, index]`.
  - `~`: Ignore minimum value (not needed).
  - `minD`: Index of nearest constellation point (1-4).
- **Line 85**: `symb_RX_estimated(k) = Lm(minD)`: Assign nearest constellation point.
  - Hard decision: quantize to nearest symbol.

**QPSK Detection Example:**
```
Received: r = 0.8 + 0.9i
Distances:
  d1 = ||r - (-1+1i)|| = ||1.8 - 0.1i|| ≈ 1.80
  d2 = ||r - (+1+1i)|| = ||-0.2 - 0.1i|| ≈ 0.22  ← minimum
  d3 = ||r - (+1-1i)|| = ||-0.2 + 1.9i|| ≈ 1.91
  d4 = ||r - (-1-1i)|| = ||1.8 + 1.9i|| ≈ 2.62
Decision: ŝ = +1+1i (index 2)
```

### Lines 89-102: 16-QAM Detection

```matlab
%% 16QAM constellation
% The 16QAM constellation is as shown below:
% 
% 
if nb_bit_per_symb == 4
    
    % Code here the symbol estimation function for 16QAM constellation (output: symb_RX_estimated)
    Lm = [-3+3i -1+3i +1+3i +3+3i -3+1i -1+1i +1+1i +3+1i -3-1i -1-1i +1-1i +3-1i -3-3i -1-3i +1-3i +3-3i];
    for k=1:nb_symb
        Ld = dist(symb_RX(k)*ones(1,16),Lm);
        [~,minD] = min(Ld);
        symb_RX_estimated(k) = Lm(minD);
    end
end
```

**Explanation:**
- **Line 89**: Section header for 16-QAM.
- **Line 93**: `if nb_bit_per_symb == 4`: Check if 16-QAM mode.
- **Line 95**: Comment: Symbol estimation for 16-QAM.
- **Line 96**: `Lm = [-3+3i -1+3i ... +3-3i]`: 16-QAM constellation points.
  - All 16 possible symbols: `I ∈ {-3,-1,+1,+3}`, `Q ∈ {-3,-1,+1,+3}`.
  - Order: Row by row (Q from +3 to -3, I from -3 to +3).
- **Line 97**: `for k=1:nb_symb`: Loop over received symbols.
- **Line 98**: `Ld = dist(symb_RX(k)*ones(1,16),Lm)`: Calculate distances.
  - Replicate received symbol 16 times.
  - Calculate distance to each of 16 constellation points.
  - Result: `Ld` is a vector of 16 distances.
- **Line 99**: `[~,minD] = min(Ld)`: Find minimum distance index (1-16).
- **Line 100**: `symb_RX_estimated(k) = Lm(minD)`: Assign nearest constellation point.

**16-QAM Detection Example:**
```
Received: r = 0.5 + 1.2i
Distances to all 16 points calculated...
Minimum distance: d_min = ||r - (+1+1i)|| ≈ 0.54
Decision: ŝ = +1+1i
```

### Lines 103-108: Helper Function: Euclidean Distance

```matlab
end
%% 
% 
function d = dist(A,B)
    d = sqrt( (real(A)-real(B)).^2 + (imag(A)-imag(B)).^2 );
end
```

**Explanation:**
- **Line 103**: End of main function.
- **Lines 106-108**: Helper function `dist(A,B)`:
  - **Line 107**: `real(A)-real(B)`: I-component difference.
  - **Line 107**: `imag(A)-imag(B)`: Q-component difference.
  - **Line 107**: `.^2`: Square both differences.
  - **Line 107**: `sqrt(...)`: Euclidean distance = `√((ΔI)² + (ΔQ)²)`.
  - **Purpose**: Calculate distance between two complex numbers (or vectors).

**Euclidean Distance Formula:**
```
d = √((I_A - I_B)² + (Q_A - Q_B)²)
   = |A - B|
```

---

## Summary

This function implements **minimum-distance (nearest-neighbor) detection**:

1. **QPSK (4-QAM)**:
   - 4 constellation points: `±1 ± 1i`.
   - Calculate distance to all 4 points.
   - Select nearest point.

2. **16-QAM**:
   - 16 constellation points: `I,Q ∈ {-3,-1,+1,+3}`.
   - Calculate distance to all 16 points.
   - Select nearest point.

**Key Points**:
- **Optimal for AWGN channels**: Minimum-distance detector maximizes probability of correct detection.
- **Hard decision**: Output is quantized to nearest constellation point (not soft values).
- **Euclidean distance**: Standard 2D distance in I-Q plane.
- This is the **inverse** of the mapping process (quantization step).

**Relationship to Other Functions**:
- **Input**: Noisy symbols from channel (after matched filter, channel estimation, Alamouti decoding).
- **Output**: Hard-decided symbols for `demapping_QAM.m` (convert to bits).

**Theoretical Background**:
- For AWGN channels with equiprobable symbols, minimum-distance detection is optimal (MAP = MLE).
- Minimizes symbol error probability.
- Equivalent to maximum likelihood detection.

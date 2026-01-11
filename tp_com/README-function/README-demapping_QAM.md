# demapping_QAM.m: Line-by-Line Code Explanation

This document provides a **comprehensive, line-by-line explanation** of every code statement in `demapping_QAM.m`, which converts hard-decided QAM symbols back to bits using Gray coding.

---

## Function Overview

**Purpose**: Convert hard-decided QAM symbols to bit stream using inverse Gray mapping.

**Inputs**:
- `symb_RX_estimated`: Hard-decided symbols (nearest constellation points)
- `nb_bit_per_symb`: Bits per symbol (2 for QPSK, 4 for 16-QAM)
- `nb_symb`: Number of symbols

**Output**:
- `bit_RX`: Estimated bit stream (column vector)

---

## Line-by-Line Explanation

### Lines 1-17: Function Header and Initialization

```matlab
function bit_RX = demapping_QAM( symb_RX_estimated , nb_bit_per_symb , nb_symb )
%% DEMAPPING_QAM Complex-symbol-to-bits Demapping for IQ Modulations
% This functions demaps a complex symbol stream into a bit stream with respect 
% to some QAM constellations using gray coding.
% 
% *Inputs:*
%% 
% * _symb_RX_estimated_: estimated symbol stream vector of size _nb_symb x 1_
% * _nb_bit_per_symb_: number of bits per symbol
% * _nb_symb_: number of symbols in the  symbol stream (= length of _symb_RX_)
%% 
% *Ouputs:*
%% 
% * _bit_RX_: estimated bit stream vector of size _nb_bit x 1_ where _nb_bit 
% = nb_bit_per_symb x nb_symb_
nb_bit = nb_symb * nb_bit_per_symb ;                  % Number of bits of the bit stream
bit_RX = zeros( nb_bit , 1 ) ;                        % Bit stream vector initialization
```

**Explanation:**
- **Line 1**: Function declaration. Returns `bit_RX` (column vector of bits).
- **Lines 2-15**: Documentation explaining:
  - Purpose: Convert symbols to bits using Gray coding.
  - Inputs: Hard-decided symbols, bits per symbol, number of symbols.
  - Output: Bit stream.
- **Line 16**: `nb_bit = nb_symb * nb_bit_per_symb`: Calculate total bits.
  - Example: `nb_symb=1000`, `nb_bit_per_symb=4` → `nb_bit=4000`.
- **Line 17**: `bit_RX = zeros(nb_bit, 1)`: Initialize output vector (all zeros).

### Lines 18-48: QPSK (4-QAM) Demapping

```matlab
%% 4QAM constellation (<=> QPSK)
% The de-mapping for the QPSK constellation should be done as shown below:
% 
% 
% 
% This mapping follows a gray coding: each symbol differs from its neighbors 
% by one bit only. This reduces bit errors as a received symbol distorted by some 
% noise is more likely to fall close to an adjacent symbol  than close to a faraway 
% symbol. 
if nb_bit_per_symb == 2
    
    % Code here the demapping function for QPSK constellation with Gray
    % coding (output: bit_RX).
    for k=1:nb_symb
        kb = 2*(k-1)+1;
        
        I = real(symb_RX_estimated(k));
        if (I == 1)
            bit_RX(kb) = 1;
        else
            bit_RX(kb) = 0;
        end
        Q = imag(symb_RX_estimated(k));
        if (Q == 1)
            bit_RX(kb+1) = 1;
        else
            bit_RX(kb+1) = 0;
        end
    end
    
end
```

**Explanation:**
- **Line 18**: Section header for QPSK (4-QAM).
- **Lines 19-26**: Comments explaining Gray coding benefits:
  - Adjacent symbols differ by one bit.
  - Reduces bit errors (noise more likely to cause adjacent symbol errors).
- **Line 27**: `if nb_bit_per_symb == 2`: Check if QPSK mode.
- **Line 29**: Comment: Demapping for QPSK with Gray coding.
- **Line 30**: `for k=1:nb_symb`: Loop over symbols.
- **Line 31**: `kb = 2*(k-1)+1`: Calculate starting bit index for symbol `k`.
  - Symbol 1: bits 1-2 (`kb=1`).
  - Symbol 2: bits 3-4 (`kb=3`).
  - Symbol 3: bits 5-6 (`kb=5`).
  - etc.
- **Line 33**: `I = real(symb_RX_estimated(k))`: Extract in-phase (real) component.
  - QPSK symbols: `±1 ± 1i`.
- **Lines 34-38**: Demap I component:
  - `if (I == 1)`: If I = +1 → bit = 1.
  - `else`: If I = -1 → bit = 0.
- **Line 40**: `Q = imag(symb_RX_estimated(k))`: Extract quadrature (imaginary) component.
- **Lines 41-45**: Demap Q component:
  - `if (Q == 1)`: If Q = +1 → bit = 1.
  - `else`: If Q = -1 → bit = 0.

**QPSK Constellation and Gray Mapping:**
```
QPSK Constellation:
        Q
        ↑
  10    |    11
  (-1+1i)  |  (1+1i)
  --------+--------→ I
  00    |    01
  (-1-1i)  |  (1-1i)
```

**Demapping Logic:**
- I = +1 → first bit = 1, I = -1 → first bit = 0.
- Q = +1 → second bit = 1, Q = -1 → second bit = 0.

### Lines 49-94: 16-QAM Demapping

```matlab
%% 16QAM constellation
% The de-mapping for the 16QAM constellation should be done as shown below:
% 
% 
% 
% This mapping follows a gray coding: each symbol differs from its neighbors 
% by one bit only. This reduces bit errors as a received symbol distorted by some 
% noise is more likely to fall close to an adjacent symbol  than close to a faraway 
% symbol. 
if nb_bit_per_symb == 4
    
    % Code here the demapping function for 16QAM constellation with Gray
    % coding (output: bit_RX)
    for k=1:nb_symb
        kb = 4*(k-1)+1;
        
        I = real(symb_RX_estimated(k));
        if (I == -3)
            bit_RX(kb) = 0;
            bit_RX(kb+1) = 0;  
        elseif (I == -1)
            bit_RX(kb) = 0;
            bit_RX(kb+1) = 1;  
        elseif (I == +1)
            bit_RX(kb) = 1;
            bit_RX(kb+1) = 1; 
        else
            bit_RX(kb) = 1;
            bit_RX(kb+1) = 0; 
        end
        Q = imag(symb_RX_estimated(k));
        if (Q == -3)
            bit_RX(kb+2) = 1;
            bit_RX(kb+3) = 0;  
        elseif (Q == -1)
            bit_RX(kb+2) = 1;
            bit_RX(kb+3) = 1;  
        elseif (Q == +1)
            bit_RX(kb+2) = 0;
            bit_RX(kb+3) = 1; 
        else
            bit_RX(kb+2) = 0;
            bit_RX(kb+3) = 0; 
        end
    end
end
```

**Explanation:**
- **Line 49**: Section header for 16-QAM.
- **Lines 50-57**: Comments explaining Gray coding (same as QPSK).
- **Line 58**: `if nb_bit_per_symb == 4`: Check if 16-QAM mode.
- **Line 60**: Comment: Demapping for 16-QAM with Gray coding.
- **Line 61**: `for k=1:nb_symb`: Loop over symbols.
- **Line 62**: `kb = 4*(k-1)+1`: Calculate starting bit index.
  - Symbol 1: bits 1-4 (`kb=1`).
  - Symbol 2: bits 5-8 (`kb=5`).
  - Symbol 3: bits 9-12 (`kb=9`).
  - etc.
- **Line 64**: `I = real(symb_RX_estimated(k))`: Extract I component.
  - 16-QAM I values: `-3, -1, +1, +3`.
- **Lines 65-78**: Demap I component (2 bits):
  - `I == -3` → bits `[0, 0]`.
  - `I == -1` → bits `[0, 1]`.
  - `I == +1` → bits `[1, 1]`.
  - `I == +3` → bits `[1, 0]` (else case).
- **Line 79**: `Q = imag(symb_RX_estimated(k))`: Extract Q component.
  - 16-QAM Q values: `-3, -1, +1, +3`.
- **Lines 80-93**: Demap Q component (2 bits):
  - `Q == -3` → bits `[1, 0]`.
  - `Q == -1` → bits `[1, 1]`.
  - `Q == +1` → bits `[0, 1]`.
  - `Q == +3` → bits `[0, 0]` (else case).

**16-QAM Constellation and Gray Mapping:**
```
16-QAM Constellation (I-Q plane):
        Q
        ↑
  +3 ───┼───┼───┼───
    1011│1111│0111│0011
  +1 ───┼───┼───┼───
    1010│1110│0110│0010
  -1 ───┼───┼───┼───
    1000│1100│0100│0000
  -3 ───┼───┼───┼───
    1001│1101│0101│0001
      -3  -1  +1  +3  → I
```

**Demapping Logic:**
- **I component** (bits 1-2):
  - I = -3 → `[0, 0]`
  - I = -1 → `[0, 1]`
  - I = +1 → `[1, 1]`
  - I = +3 → `[1, 0]`
- **Q component** (bits 3-4):
  - Q = -3 → `[1, 0]`
  - Q = -1 → `[1, 1]`
  - Q = +1 → `[0, 1]`
  - Q = +3 → `[0, 0]`

**Note**: The Q mapping is inverted compared to I (this is part of the Gray coding scheme).

### Line 95: End of Function

```matlab
end
```

**Explanation:**
- Marks the end of the function.

---

## Summary

This function performs **inverse Gray mapping** from QAM symbols to bits:

1. **QPSK (4-QAM)**:
   - 2 bits per symbol.
   - I component → first bit.
   - Q component → second bit.
   - Simple threshold: `±1` → `1/0`.

2. **16-QAM**:
   - 4 bits per symbol.
   - I component → first 2 bits (mapping: -3→00, -1→01, +1→11, +3→10).
   - Q component → last 2 bits (mapping: -3→10, -1→11, +1→01, +3→00).

**Key Points**:
- This is the **inverse** of `mapping_QAM.m`.
- Input symbols are **hard-decided** (already quantized to nearest constellation point).
- Gray coding ensures adjacent symbols differ by one bit (reduces bit error rate).
- The demapping is deterministic (no ambiguity for hard-decided symbols).

**Relationship to Other Functions**:
- **Input**: Hard-decided symbols from `symbol_estimation_QAM.m`.
- **Output**: Bit stream for BER calculation (compare with original transmitted bits).

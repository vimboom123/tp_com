# mapping_QAM.m: Line-by-Line Code Explanation

This document provides a **comprehensive, line-by-line explanation** of every code statement in `mapping_QAM.m`, which converts bit streams to QAM symbols using Gray coding.

---

## Function Overview

**Purpose**: Map bit stream to complex QAM symbols using Gray-coded constellation.

**Inputs**:
- `bit_TX`: Bit stream vector (column vector of 0s and 1s)
- `nb_bit_per_symb`: Number of bits per symbol (2 for QPSK, 4 for 16-QAM)
- `nb_bit`: Total number of bits (should equal `length(bit_TX)`)

**Output**:
- `symb_TX`: Complex symbol stream (column vector)

---

## Line-by-Line Explanation

### Lines 1-20: Function Header and Validation

```matlab
function symb_TX = mapping_QAM( bit_TX , nb_bit_per_symb , nb_bit )
%% MAPPING_QAM Bit-to-complex-symbol mapping for IQ Modulations
% This functions maps a bit stream into a complex symbol stream with respect 
% to some IQ constellations.
% 
% *Inputs:*
%% 
% * _bit_TX_: bit stream vector of size _nb_bit x 1_
% * _nb_bit_per_symb_: number of bits per symbol
% * _nb_bit_: number of bits in the input bit stream (= length of _bit_TX_)
%% 
% *Ouputs:*
%% 
% * _symb_TX_: complex symbol stream vector of size _nb_symb x 1_ where _nb_symb 
% = nb_bit / nb_bit_per_symb_
if rem( nb_bit , nb_bit_per_symb ) ~= 0
    error('issue: the data stream is not an integer number of symbols')
end
nb_symb = nb_bit / nb_bit_per_symb ;                                % Number of symbols of the data stream
symb_TX = zeros( nb_symb , 1 ) ;                                    % Symbol stream vector initialization
```

**Explanation:**
- **Line 1**: Function declaration. Returns `symb_TX` (column vector of complex symbols).
- **Lines 2-15**: Documentation explaining:
  - Purpose: Map bits to QAM symbols.
  - Inputs: Bit stream, bits per symbol, total bits.
  - Output: Symbol stream.
- **Line 16**: `if rem(nb_bit, nb_bit_per_symb) ~= 0`: Check if bit count is divisible by bits per symbol.
  - `rem(a, b)`: Remainder of `a/b`.
  - If remainder ≠ 0, bit stream cannot form complete symbols.
- **Line 17**: `error(...)`: Throw error if validation fails.
  - Example: `nb_bit=1001`, `nb_bit_per_symb=4` → error (1001/4 = 250.25).
- **Line 19**: `nb_symb = nb_bit / nb_bit_per_symb`: Calculate number of symbols.
  - Example: `nb_bit=4000`, `nb_bit_per_symb=4` → `nb_symb=1000`.
- **Line 20**: `symb_TX = zeros(nb_symb, 1)`: Initialize output vector (all zeros).

### Lines 21-38: QPSK (4-QAM) Mapping

```matlab
%% 4QAM constellation (<=> QPSK)
% The mapping for the QPSK constellation should be done as shown below:
% 
% 
% 
% This mapping follows a gray coding: each symbol differs from its neighbors 
% by one bit only. This reduces bit errors as a received symbol distorted by some 
% noise is more likely to fall close to an adjacent symbol  than close to a faraway 
% symbol. 
if nb_bit_per_symb == 2
    
    % Code here the mapping function for QPSK constellation with Gray
    % coding (output: symb_TX)
    for k=1:nb_symb
        kb = 2*(k-1)+1;
        symb_TX(k) = (2*bit_TX(kb)-1) + 1i*(2*bit_TX(kb+1)-1);
    end
end
```

**Explanation:**
- **Line 21**: Section header for QPSK (4-QAM).
- **Lines 22-29**: Comments explaining Gray coding:
  - Adjacent symbols differ by one bit.
  - Reduces bit errors (noise more likely to cause adjacent symbol errors).
- **Line 30**: `if nb_bit_per_symb == 2`: Check if QPSK mode.
- **Line 32**: Comment: Mapping for QPSK with Gray coding.
- **Line 33**: `for k=1:nb_symb`: Loop over symbols.
- **Line 34**: `kb = 2*(k-1)+1`: Calculate starting bit index for symbol `k`.
  - Symbol 1: bits 1-2 (`kb=1`).
  - Symbol 2: bits 3-4 (`kb=3`).
  - Symbol 3: bits 5-6 (`kb=5`).
  - etc.
- **Line 35**: `symb_TX(k) = (2*bit_TX(kb)-1) + 1i*(2*bit_TX(kb+1)-1)`: Map 2 bits to QPSK symbol.
  - **I component**: `(2*bit_TX(kb)-1)`
    - Bit = 0 → `2×0-1 = -1`.
    - Bit = 1 → `2×1-1 = +1`.
  - **Q component**: `1i*(2*bit_TX(kb+1)-1)`
    - Bit = 0 → `1i×(-1) = -1i`.
    - Bit = 1 → `1i×(+1) = +1i`.
  - **Result**: Symbol = I + jQ = `±1 ± 1i`.

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

**Mapping Table:**
- `[0, 0]` → `-1 - 1i`
- `[0, 1]` → `-1 + 1i`
- `[1, 1]` → `+1 + 1i`
- `[1, 0]` → `+1 - 1i`

### Lines 39-77: 16-QAM Mapping

```matlab
%% 16QAM constellation
% The mapping for the 16QAM constellation should be done as shown below:
% 
% 
% 
% This mapping follows a gray coding: each symbol differs from its neighbors 
% by one bit only. This reduces bit errors as a received symbol distorted by some 
% noise is more likely to fall close to an adjacent symbol  than close to a faraway 
% symbol. 
if nb_bit_per_symb == 4
    
    % Code here the mapping function for 16QAM constellation with Gray
    % coding (output: symb_TX)
    for k=1:nb_symb
        kb = 4*(k-1)+1;
        
        I = [bit_TX(kb) bit_TX(kb+1)];
        if (I == [0 0])
            Ib = -3;
        elseif (I == [0 1])
            Ib = -1;
        elseif (I == [1 1])
            Ib = +1;
        else
            Ib = +3;
        end
        Q = [bit_TX(kb+2) bit_TX(kb+3)];
        if (Q == [0 0])
            Qb = +3;
        elseif (Q == [0 1])
            Qb = +1;
        elseif (Q == [1 1])
            Qb = -1;
        else
            Qb = -3;
        end
        symb_TX(k) = Ib + 1i*Qb;
    end
end
```

**Explanation:**
- **Line 39**: Section header for 16-QAM.
- **Lines 40-47**: Comments explaining Gray coding (same as QPSK).
- **Line 48**: `if nb_bit_per_symb == 4`: Check if 16-QAM mode.
- **Line 50**: Comment: Mapping for 16-QAM with Gray coding.
- **Line 51**: `for k=1:nb_symb`: Loop over symbols.
- **Line 52**: `kb = 4*(k-1)+1`: Calculate starting bit index.
  - Symbol 1: bits 1-4 (`kb=1`).
  - Symbol 2: bits 5-8 (`kb=5`).
  - Symbol 3: bits 9-12 (`kb=9`).
  - etc.
- **Line 54**: `I = [bit_TX(kb) bit_TX(kb+1)]`: Extract first 2 bits (I component).
  - Bits `kb` and `kb+1` determine I value.
- **Lines 55-63**: Map I bits to I coordinate:
  - `[0, 0]` → `Ib = -3`.
  - `[0, 1]` → `Ib = -1`.
  - `[1, 1]` → `Ib = +1`.
  - `[1, 0]` → `Ib = +3` (else case).
- **Line 64**: `Q = [bit_TX(kb+2) bit_TX(kb+3)]`: Extract last 2 bits (Q component).
  - Bits `kb+2` and `kb+3` determine Q value.
- **Lines 65-73**: Map Q bits to Q coordinate:
  - `[0, 0]` → `Qb = +3`.
  - `[0, 1]` → `Qb = +1`.
  - `[1, 1]` → `Qb = -1`.
  - `[1, 0]` → `Qb = -3` (else case).
- **Line 74**: `symb_TX(k) = Ib + 1i*Qb`: Combine I and Q into complex symbol.
  - Result: `symb_TX(k) = Ib + j·Qb`.

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

**Mapping Table (I component, bits 1-2):**
- `[0, 0]` → I = -3
- `[0, 1]` → I = -1
- `[1, 1]` → I = +1
- `[1, 0]` → I = +3

**Mapping Table (Q component, bits 3-4):**
- `[0, 0]` → Q = +3
- `[0, 1]` → Q = +1
- `[1, 1]` → Q = -1
- `[1, 0]` → Q = -3

**Note**: Q mapping is inverted compared to I (this is part of the Gray coding scheme).

### Line 78: End of Function

```matlab
end
```

**Explanation:**
- Marks the end of the function.

---

## Summary

This function performs **Gray-coded QAM mapping** from bits to symbols:

1. **QPSK (4-QAM)**:
   - 2 bits per symbol.
   - I = `±1`, Q = `±1`.
   - Simple formula: `(2b-1) + j(2b-1)`.

2. **16-QAM**:
   - 4 bits per symbol (2 bits for I, 2 bits for Q).
   - I = `{-3, -1, +1, +3}`, Q = `{-3, -1, +1, +3}`.
   - Lookup table mapping with Gray coding.

**Key Points**:
- Gray coding ensures adjacent symbols differ by one bit (reduces bit error rate).
- I and Q are mapped independently (simplifies implementation).
- The constellation is unnormalized (points are `±1, ±3` for 16-QAM).
- This is the **inverse** of `demapping_QAM.m`.

**Relationship to Other Functions**:
- **Input**: Random bits from `randi([0 1], ...)`.
- **Output**: QAM symbols for transmission (input to `convolution_TX.m`).

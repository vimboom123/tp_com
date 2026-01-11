# generate_channel.m: Line-by-Line Code Explanation

This document provides a **comprehensive, line-by-line explanation** of every code statement in `generate_channel.m`, which generates Rayleigh and Rician (Rice) fading channel coefficients.

---

## Function Overview

**Purpose**: Generate channel realizations following Rayleigh or Rician distributions for wireless channel modeling.

**Inputs**:
- `nb_realization`: Number of channel realizations (scalar)
- `channel_type`: Channel type string (`'Rayleigh'` or `'Rice'`)
- `K`: Rice factor (scalar, only used for Rice channel)

**Output**:
- `H`: Channel response vector of size `1 × nb_realization`

---

## Line-by-Line Explanation

### Lines 1-17: Function Header and Documentation

```matlab
function H = generate_channel( nb_realization , channel_type , K )
%% GENERATE_CHANNEL Generation of the channel model
% This function implements the Rayleigh and Rice channel models. The details 
% of such models can be found in the chapter 2 of the course of Philippe De Doncker 
% (see especially section 2.2.2.).
% 
% *Inputs:*
%% 
% * _nb_realization_: indicates the number of realizations of the channel (scalar 
% quantity)
% * _channel_type_: indicates the type of channel to generate (string quantity). 
% Can take 2 values: _Rayleigh, Rice_
% * _K:_ Rice factor (scalar quantity)
%% 
% *Ouput:*
%% 
% * _channel_: channel response (vector of size _1 x nb_realization_)
```

**Explanation:**
- **Line 1**: Function declaration. Returns `H` (row vector of channel coefficients).
- **Lines 2-17**: Documentation explaining:
  - Purpose: Generate Rayleigh or Rice channel models.
  - Input `nb_realization`: Number of independent channel samples.
  - Input `channel_type`: String selector (`'Rayleigh'` or `'Rice'`).
  - Input `K`: Rice factor (LOS to NLOS power ratio).
  - Output: Channel vector.

### Line 18: Switch Statement

```matlab
switch channel_type
```

**Explanation:**
- **Line 18**: Switch based on channel type to select generation method.

### Lines 19-38: Rayleigh Channel Generation

```matlab
%% Rayleigh channel (_N_-waves model)
% In a rich multipath Non-Line-Of-Sight (NLOS) environment, the propagation 
% channel can be well modeled by a random variable following a complex normal 
% distribution such as:
% 
% $$h_{Rayleigh}\sim\mathit{C} \mathcal{N}(0,1)$$ 
% 
% In this case, the channel magnitude $|h_{Rayleigh}|$has a Rayleigh distribution 
% and the channel phase $\angle h_{Rayleigh}$ has a uniform distribution over 
% $2\pi$.
% 
% In this code, the following normalization, $\mathrm{E}\left[ |h_{Rayleigh}|^2 
% \right]=1$, should be observed (i.e., the variance of both the real and imaginary 
% parts of $h_{Rayleigh}$ is $\sigma^2=\frac{1}{2}$).
    case 'Rayleigh'
        % Code the Rayleigh channel here
        H_Ray_mag = raylrnd(1,1,nb_realization);
        H_Ray_mag = H_Ray_mag / sqrt( (sum(H_Ray_mag.^2)/nb_realization) );
        H_Ray_ph = 2*pi*rand(1,nb_realization);
        H = H_Ray_mag .* exp(1i*H_Ray_ph);
```

**Explanation:**
- **Lines 19-32**: Documentation explaining Rayleigh channel:
  - **Physical model**: Rich multipath NLOS environment (no direct path).
  - **Statistical model**: Complex Gaussian `h ~ CN(0,1)`.
  - **Magnitude**: Rayleigh distributed `|h| ~ Rayleigh`.
  - **Phase**: Uniform over `[0, 2π]`.
  - **Normalization**: `E[|h|²] = 1` (unit average power).
- **Line 33**: `case 'Rayleigh'`: Handle Rayleigh channel type.
- **Line 35**: `H_Ray_mag = raylrnd(1,1,nb_realization)`: Generate Rayleigh magnitudes.
  - `raylrnd(1, ...)`: Rayleigh distribution with scale parameter 1.
  - Output: `1 × nb_realization` vector of magnitudes.
  - **Note**: This generates `|h|`, not the full complex channel.
- **Line 36**: `H_Ray_mag = H_Ray_mag / sqrt( (sum(H_Ray_mag.^2)/nb_realization) )`: Normalize to unit average power.
  - `sum(H_Ray_mag.^2)/nb_realization`: Average squared magnitude.
  - `sqrt(...)`: Square root to get RMS magnitude.
  - Division: Normalize so `E[|h|²] = 1`.
  - **Why**: Ensures consistent channel power across realizations.
- **Line 37**: `H_Ray_ph = 2*pi*rand(1,nb_realization)`: Generate uniform phases.
  - `rand(1,nb_realization)`: Uniform random numbers in `[0, 1]`.
  - `2*pi*`: Scale to `[0, 2π]`.
  - Output: `1 × nb_realization` vector of phases.
- **Line 38**: `H = H_Ray_mag .* exp(1i*H_Ray_ph)`: Combine magnitude and phase.
  - `exp(1i*H_Ray_ph)`: Complex exponential (unit magnitude, random phase).
  - `.*`: Element-wise multiplication.
  - Result: `H = |h|·e^(jφ)` (complex channel with Rayleigh magnitude, uniform phase).

**Rayleigh Channel Model:**
```
h = |h| · e^(jφ)
where:
  |h| ~ Rayleigh(σ)  (magnitude)
  φ ~ Uniform(0, 2π)  (phase)
  E[|h|²] = 1  (normalization)
```

### Lines 39-72: Rice Channel Generation

```matlab
%% Rice channel (_1+N_-waves model)
% In a rich multipath environment, if one multipath components (MPC) dominates 
% the other multipath, the propagation channel can be well modeled by:
% 
% $$h_{Rice}=\sqrt{\frac{K}{1+K}}h_{strong MPC}+\sqrt{\frac{1}{1+K}}h_{Rayleigh}$$
% 
% where:
%% 
% * $h_{Rayleigh}\sim\mathit{C} \mathcal{N}(0,1)$ with the following normalization 
% $\mathrm{E}\left[ |h_{Rayleigh}|^2 \right]=1$ (i.e., the variance of both the 
% real and imaginary parts of $h_{Rayleigh}$ is $\sigma^2=\frac{1}{2}$).
% * $h_{strong MPC}=e^{j\phi}$ is a deterministic magnitude and phase value 
% of a strong MPC. Here the magnitude is 1 so that the amplitude of the strong 
% MPC is completely characterized with the Rice factor K.
% * $K=\frac{ |h_{strong MPC}|^2 }{\mathrm{E}\left[ |h_{NLOS}|^2 \right]}$ is 
% the Rice factor
% * $|h_{Rice}|$has a Rice distribution.
%% 
% Typcally, the strong MPC can correspond to the Line-Of-Sight (LOS) contribution 
% or to a strong reflection on a large wall for instance, whereas the second term 
% $h_{Rayleigh}$ corresponds to the NLOS contributions.
% 
% *Tip:* You can put any specific value for $\phi$ and observe the influence 
% on the results in the main.
    case 'Rice'
        % Code the Rice channel here
        H_Ray_mag = raylrnd(1,1,nb_realization);
        H_Ray_mag = H_Ray_mag / sqrt( (sum(H_Ray_mag.^2)/nb_realization) );
        H_Ray_ph = 2*pi*rand(1,nb_realization);
        H_Ray = H_Ray_mag .* exp(1i*H_Ray_ph);
        phy = 1;
        H_MPC = ones(1,nb_realization)*1*exp(1i*phy);
        
        H = sqrt(K/(1+K)).*H_MPC + sqrt(1/(1+K)).*H_Ray;
```

**Explanation:**
- **Lines 39-62**: Documentation explaining Rice channel:
  - **Physical model**: One dominant path (LOS or strong reflection) + scattered paths.
  - **Mathematical model**: `h = √(K/(1+K))·h_LOS + √(1/(1+K))·h_Rayleigh`.
  - **K-factor**: Ratio of LOS power to NLOS power.
    - `K = 0`: Pure Rayleigh (no LOS).
    - `K → ∞`: Pure LOS (no fading).
  - **Components**:
    - `h_LOS = e^(jφ)`: Deterministic LOS component (unit magnitude, fixed phase).
    - `h_Rayleigh`: Scattered component (complex Gaussian).
- **Line 63**: `case 'Rice'`: Handle Rice channel type.
- **Lines 65-68**: Generate Rayleigh (NLOS) component (same as Rayleigh case):
  - **Line 65**: Generate Rayleigh magnitudes.
  - **Line 66**: Normalize to unit average power.
  - **Line 67**: Generate uniform phases.
  - **Line 68**: Combine: `H_Ray = |h|·e^(jφ)`.
- **Line 69**: `phy = 1`: Set LOS phase to 1 radian.
  - **Note**: This is a fixed phase (deterministic).
  - **Tip**: Can be changed to observe phase effects.
- **Line 70**: `H_MPC = ones(1,nb_realization)*1*exp(1i*phy)`: Generate LOS component.
  - `ones(1,nb_realization)`: Vector of ones (length `nb_realization`).
  - `*1`: Unit magnitude (can be changed).
  - `*exp(1i*phy)`: Complex exponential with phase `phy`.
  - Result: `H_MPC = e^(j·1)` (constant across all realizations).
- **Line 72**: `H = sqrt(K/(1+K)).*H_MPC + sqrt(1/(1+K)).*H_Ray`: Combine LOS and NLOS.
  - **First term**: `√(K/(1+K))·H_MPC` (LOS component, weighted by K).
  - **Second term**: `√(1/(1+K))·H_Ray` (NLOS component, weighted by 1).
  - **Power normalization**: `K/(1+K) + 1/(1+K) = 1` (total power = 1).
  - **Element-wise**: `.*` ensures vectorized operation.

**Rice Channel Model:**
```
h = √(K/(1+K)) · h_LOS + √(1/(1+K)) · h_Rayleigh

where:
  h_LOS = e^(jφ)  (deterministic LOS, unit magnitude)
  h_Rayleigh ~ CN(0,1)  (scattered component)
  K = |h_LOS|² / E[|h_Rayleigh|²]  (Rice factor)
```

**Power Distribution:**
- LOS power: `K/(1+K)`.
- NLOS power: `1/(1+K)`.
- Total power: `1` (normalized).

---

## Summary

This function generates two types of fading channels:

1. **Rayleigh Channel**:
   - Pure NLOS (no direct path).
   - Magnitude: Rayleigh distributed.
   - Phase: Uniform over `[0, 2π]`.
   - Normalized to `E[|h|²] = 1`.

2. **Rice Channel**:
   - LOS + NLOS components.
   - Formula: `h = √(K/(1+K))·h_LOS + √(1/(1+K))·h_Rayleigh`.
   - K-factor controls LOS/NLOS ratio.
   - Normalized to total power = 1.

**Key Points**:
- Both channels are normalized to unit average power.
- Rayleigh is a special case of Rice (K = 0).
- The Rice LOS component has fixed phase (can be modified).
- These models are fundamental for wireless communication simulation.

**Relationship to Other Functions**:
- Used in base project (`Main_Alamouti_Full_Project.m`) for channel generation.
- Extended version (`gen_miso_2x1_channel_samples.m`) adds spatial correlation and time variation.

# Function Documentation Index

This folder contains detailed, line-by-line code explanations for all helper functions used in the Alamouti STBC project.

## Available Documentation

1. **[README-convolution_RX.md](README-convolution_RX.md)**
   - Receiver-side matched filter and downsampling
   - Convolves received signal with RRC filter
   - Downsamples from sample rate to symbol rate

2. **[README-convolution_TX.md](README-convolution_TX.md)**
   - Transmitter-side pulse shaping and upsampling
   - Upsamples symbols and applies RRC filter
   - Produces bandlimited waveform for transmission

3. **[README-demapping_QAM.md](README-demapping_QAM.md)**
   - Converts hard-decided QAM symbols to bits
   - Implements inverse Gray mapping
   - Supports QPSK (4-QAM) and 16-QAM

4. **[README-generate_channel.md](README-generate_channel.md)**
   - Generates Rayleigh and Rician fading channels
   - Implements channel models for wireless simulation
   - Normalized to unit average power

5. **[README-mapping_QAM.md](README-mapping_QAM.md)**
   - Converts bit stream to QAM symbols
   - Implements Gray-coded constellation mapping
   - Supports QPSK (4-QAM) and 16-QAM

6. **[README-symbol_estimation_QAM.md](README-symbol_estimation_QAM.md)**
   - Minimum-distance (nearest-neighbor) symbol detection
   - Optimal for AWGN channels
   - Hard decision quantization

7. **[README-raised_cosine.md](README-raised_cosine.md)**
   - Designs Raised Cosine (RC) and Root Raised Cosine (RRC) filters
   - Pulse shaping filter impulse response generation
   - Handles indeterminate forms and normalization

## Documentation Format

Each README file provides:
- **Function Overview**: Purpose, inputs, outputs
- **Line-by-Line Explanation**: Detailed explanation of every code statement
- **Mathematical Formulation**: Theoretical background where applicable
- **Summary**: Key points and relationships to other functions

## Usage

These documents are designed to help understand:
- What each function does
- How each line of code works
- Why certain implementation choices were made
- How functions relate to each other in the overall system

## Related Documentation

- **Main Project**: See `README.md` in parent directory for overall project structure
- **Extensions**: See `extension.md` for detailed explanation of `Main_Alamouti_Extensions.m`

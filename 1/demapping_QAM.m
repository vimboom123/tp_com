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
    % coding (output: bit_RX) % QPSK constellation with Gray coding
    % Demapping table (reverse of mapping):
    % 1+1j -> 00
    % -1+1j -> 01
    % 1-1j -> 10
    % -1-1j -> 11
    
    % Normalized constellation points
    constellation = [1+1j, -1+1j, 1-1j, -1-1j] / sqrt(2);
    
    % Bit patterns corresponding to each constellation point
    bit_patterns = [0 0;  % for 1+1j
                   0 1;  % for -1+1j
                   1 0;  % for 1-1j
                   1 1]; % for -1-1j
    
    % Process each symbol
    for i = 1:nb_symb
        % Find which constellation point this symbol corresponds to
        % (It should match exactly since it's already been estimated)
        distances = abs(symb_RX_estimated(i) - constellation);
        [~, symbol_idx] = min(distances);
        
        % Extract corresponding bits
        bit_index = (i-1)*2 + 1;
        bit_RX(bit_index:bit_index+1) = bit_patterns(symbol_idx, :)';
    end
    
end
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
    % 16QAM constellation with Gray coding
    % Build the same constellation as in mapping
    constellation = zeros(16, 1);
    bit_patterns = zeros(16, 4);
    
    % Standard 16QAM constellation
    I_mapping = [-3, -1, +3, +1];  % for bits 00, 01, 10, 11
    Q_mapping = [-3, -1, +3, +1];  % for bits 00, 01, 10, 11
    
    % Build constellation and corresponding bit patterns
    for i = 0:15
        % Original bit pattern
        bits = [floor(i/8), mod(floor(i/4),2), mod(floor(i/2),2), mod(i,2)];
        bit_patterns(i+1, :) = bits;
        
        b3b2 = floor(i/4);  % Most significant 2 bits (I-axis)
        b1b0 = mod(i, 4);   % Least significant 2 bits (Q-axis)
        
        % Gray code mapping
        I_gray = bitxor(b3b2, floor(b3b2/2));
        Q_gray = bitxor(b1b0, floor(b1b0/2));
        
        constellation(i+1) = I_mapping(I_gray+1) + 1j*Q_mapping(Q_gray+1);
    end
    
    % Normalize to unit average power
    constellation = constellation / sqrt(10);
    
    % Process each symbol
    for i = 1:nb_symb
        % Find which constellation point this symbol corresponds to
        distances = abs(symb_RX_estimated(i) - constellation);
        [~, symbol_idx] = min(distances);
        
        % Extract corresponding bits
        bit_index = (i-1)*4 + 1;
        bit_RX(bit_index:bit_index+3) = bit_patterns(symbol_idx, :)';
    end
end
end
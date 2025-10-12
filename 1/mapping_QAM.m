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
     % QPSK constellation with Gray coding
    % Bit mapping:
    % 00 -> 1+1j
    % 01 -> -1+1j  
    % 10 -> 1-1j
    % 11 -> -1-1j
    
    % QPSK constellation points (normalized)
    constellation = [1+1j, -1+1j, 1-1j, -1-1j];
    
    % Process each symbol
    for i = 1:nb_symb
        % Extract 2 bits for current symbol
        bit_index = (i-1)*2 + 1;
        bits = bit_TX(bit_index:bit_index+1);
        
        % Convert 2 bits to decimal index (0-3)
        symbol_index = bits(1)*2 + bits(2) + 1;  % +1 for MATLAB indexing
        
        % Map to constellation point
        symb_TX(i) = constellation(symbol_index);
    end
    
    % Normalize power to 1
    symb_TX = symb_TX / sqrt(2);
end
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
     % 16QAM constellation with Gray coding
    % Using standard 16QAM constellation with Gray mapping
    
    % Define 16QAM constellation points (4x4 grid)
    % Gray coded mapping for minimum bit errors
    constellation = zeros(16, 1);
    
    % Standard 16QAM constellation with Gray coding
    % Bits: b3 b2 b1 b0 -> I-axis uses b3b2, Q-axis uses b1b0
    
    % Mapping table (Gray coded):
    % I-axis: 00->-3, 01->-1, 11->+1, 10->+3
    % Q-axis: 00->-3, 01->-1, 11->+1, 10->+3
    
    I_mapping = [-3, -1, +3, +1];  % for bits 00, 01, 10, 11
    Q_mapping = [-3, -1, +3, +1];  % for bits 00, 01, 10, 11
    
    % Build constellation
    for i = 0:15
        b3b2 = floor(i/4);  % Most significant 2 bits (I-axis)
        b1b0 = mod(i, 4);   % Least significant 2 bits (Q-axis)
        
        % Gray code mapping
        I_gray = bitxor(b3b2, floor(b3b2/2));
        Q_gray = bitxor(b1b0, floor(b1b0/2));
        
        constellation(i+1) = I_mapping(I_gray+1) + 1j*Q_mapping(Q_gray+1);
    end
    
    % Process each symbol
    for i = 1:nb_symb
        % Extract 4 bits for current symbol
        bit_index = (i-1)*4 + 1;
        bits = bit_TX(bit_index:bit_index+3);
        
        % Convert 4 bits to decimal index (0-15)
        symbol_index = bits(1)*8 + bits(2)*4 + bits(3)*2 + bits(4) + 1;  % +1 for MATLAB indexing
        
        % Map to constellation point
        symb_TX(i) = constellation(symbol_index);
    end
    
    % Normalize average power to 1
    symb_TX = symb_TX / sqrt(10);  % Normalization factor for 16QAM
end
end
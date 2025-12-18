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
end
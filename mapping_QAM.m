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
    for k=1:nb_symb
        kb = 2*(k-1)+1;
        symb_TX(k) = (2*bit_TX(kb)-1) + 1i*(2*bit_TX(kb+1)-1);
    end
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
end
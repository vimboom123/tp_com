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
%% 4QAM constellation (<=> QPSK)
% The QPSK constellation is as shown below:
% 
% 
if nb_bit_per_symb == 2
    % QPSK constellation points (normalized)
    % Same constellation as in mapping function
    constellation = [1+1j, -1+1j, 1-1j, -1-1j] / sqrt(2);
    
    % For each received symbol, find the nearest constellation point
    for i = 1:nb_symb
        % Calculate Euclidean distance to each constellation point
        distances = abs(symb_RX(i) - constellation);
        
        % Find the index of minimum distance
        [~, min_idx] = min(distances);
        
        % Assign the nearest constellation point
        symb_RX_estimated(i) = constellation(min_idx);
    end
    % Code here the symbol estimation function for QPSK constellation (output: symb_RX_estimated)
    
end
%% 16QAM constellation
% The 16QAM constellation is as shown below:
% 
% 
if nb_bit_per_symb == 4
     % 16QAM constellation points (same as in mapping function)
    % Build the constellation with Gray coding
    constellation = zeros(16, 1);
    
    % Standard 16QAM constellation
    I_mapping = [-3, -1, +3, +1];  % for bits 00, 01, 10, 11
    Q_mapping = [-3, -1, +3, +1];  % for bits 00, 01, 10, 11
    
    % Build constellation with same Gray coding as mapping
    for i = 0:15
        b3b2 = floor(i/4);  % Most significant 2 bits (I-axis)
        b1b0 = mod(i, 4);   % Least significant 2 bits (Q-axis)
        
        % Gray code mapping
        I_gray = bitxor(b3b2, floor(b3b2/2));
        Q_gray = bitxor(b1b0, floor(b1b0/2));
        
        constellation(i+1) = I_mapping(I_gray+1) + 1j*Q_mapping(Q_gray+1);
    end
    
    % Normalize to unit average power
    constellation = constellation / sqrt(10);
    
    % For each received symbol, find the nearest constellation point
    for i = 1:nb_symb
        % Calculate Euclidean distance to each constellation point
        distances = abs(symb_RX(i) - constellation);
        
        % Find the index of minimum distance
        [~, min_idx] = min(distances);
        
        % Assign the nearest constellation point
        symb_RX_estimated(i) = constellation(min_idx);
    end
end
    
    % Code here the symbol estimation function for 16QAM constellation (output: symb_RX_estimated)end
end
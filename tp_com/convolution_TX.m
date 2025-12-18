function signal = convolution_TX( symb_TX , g , sps )
%% CONVOLUTION_TX Convolution (and upsampling)
% This function implements a disrete convolution between a symbol stream and 
% the impulse response of the TX filter (i.e., the pulse shape). To do so, the 
% symbol stream is first upsampled in order to match the sampling rate defined 
% by the pulse IR, and then the discrete convolution is performed).
% 
% *Inputs:*
%% 
% * _symb_TX_: complex symbol stream vector of size _nb_symb x 1_ where _nb_symb 
% =_ length( _symb_TX_ )
% * _g_: Impulse Response (IR) of the pulse of size _1 x_ length( _g_ )
% * _sps_: number of samples per symbol 
%% 
% *Ouput:*
%% 
% * _signal_: result of the convolution, vector of size _[ ( nb_symb - 1 ) * 
% sps + length( g ) ] x 1_
%% Upsampling
% In order to match the sampling rate used to design the filter impulse response 
% _g_, the symbol vector _symb_TX_ must first be upsampled as illustrated below 
% (zero-padding):
% 
% 
% Code the upsampling here
nb_symb = length( symb_TX );
nb_sample = nb_symb*sps - sps + 1;
UPsymb_TX = zeros(nb_sample,1);
for i=1:nb_symb
    UPsymb_TX( (i-1)*sps + 1 ) = symb_TX(i);
end
%% Discrete Convolution
% The discrete convolution between the upsampled symbol stream $ s_{up}$ and 
% the filter Impulse Response (IR) $g$ is defined as $\left( s_{up} * g \right)[n]=\sum_{l=-\infty}^{+\infty}s_{up}[n-l]g[l]$. 
% In our case, $g$ represents the filter Impulse Response (IR) which is finite 
% (FIR filter). So the convolution can be written as a finite summation: 
% 
% $$\left( s_{up} * g \right)[n]=\sum_{l=-L}^{L}s_{up}[n-l]g[l]$$
% 
% with $n \in [1 : nb_{sample}]$ and $2L+1$ is the number of samples of the 
% filter IR.
% 
% To implement this finite summation in Matlab, a special care to vector indexes 
% should be observed. Indeed, Matlab does not handle negative indexes. So the 
% summation should be coded as:
% 
% $$\left( s_{up} * g \right)[n]=\sum_{l=-L}^{L}s_{up}[n-l]g[l+L+1]$$
% 
% An issue also raises when:
%% 
% * $n\leq l$: indexes become negative in $ s_{up}$
% * $n - l>nb_{sample}$: indexes exceed the number of elements in $ s_{up}$
%% 
% To avoid that, it is possible to fill the $ s_{up}$ vector with $2L$ zeros 
% at the begining and $2L$ zeros at the end such as:
% 
% $ s^{filled}_{up}=[ 0~\cdots~0~s_{up}~0~\cdots~0]$. The convolution to implement 
% becomes then:
% 
% $$\left(  s^{filled}_{up} * g \right)[n]=\sum_{l=-L}^{L} s^{filled}_{up}[n-l]g[l+L+1]$$
% 
% with $n \in [L+1 : nb_{sample}+3L]$.
% Code the discrete convolution here
Lg = length(g);
L = (Lg-1)/2;
UPsymb_TX_filled = [zeros(2*L,1); UPsymb_TX; zeros(2*L,1)];
conv_sig_TX = zeros(nb_sample+4*L,1);
for i=L+1:nb_sample+3*L
    tp_sum = 0;
    for j=-L:L
        tp_sum = tp_sum + UPsymb_TX_filled(i-j)*g(j+L+1);
    end
    conv_sig_TX(i) = tp_sum;
end
% Vector cropping
% The vector resulting from the convolution of the filter IR and the symbol 
% stream contains zeros at its begining and its end. They should be removed to 
% obtain the appropriate vector size of _[ ( nb_symb - 1 ) * sps + length( g ) 
% ] x 1_.
% Crop the resulting vector here
% tail = (nb_sample+2*L) - (nb_sample+Lg-1);
% signal = conv_sig_TX(tail/2 +1:end-tail/2);
signal = conv_sig_TX(L+1:nb_sample+3*L);
%% 
% The size of the resulting signal, is equal to the upsampled stream PLUS the 
% filter length - 1. There is therefore a tail at the begining and the end of 
% the time domain vector introduced by the filter delay.
end
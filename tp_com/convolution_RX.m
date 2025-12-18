function symb_RX = convolution_RX( signal , g , sps )
%% CONVOLUTION_RX Convolution (and downsampling)
% This function implements a disrete convolution between a waveform and the 
% impulse response of the RX filter. The resulting vector is then cropped and 
% downsampled in order to retrieve the RX symbol vector.
% 
% *Inputs:*
%% 
% * _signal_: signal vector of size _[ ( nb_symb - 1 ) * sps + length( g ) ] 
% x 1_
% * _g_: Impulse Response (IR) of the pulse of size _1 x_ length( _g_ )
% * _sps_: number of samples per symbol 
%% 
% *Ouput:*
%% 
% * _symb_RX_: filtered signal which corresponds to received symbols (column 
% vector)
%% Discrete Convolution
% This part is simliar to the "Discrete Convolution" section of the function 
% "convolution_TX" and can therefore be directly copy-pasted (only variable names 
% need to be adjusted)
% Code the discrete convolution here
Lg = length(g);
L = (Lg-1)/2;
nb_sample = length(signal);
UPsymb_RX_filled = [zeros(2*L,1); signal; zeros(2*L,1)];
conv_sig_RX = zeros(nb_sample+4*L,1);
for i=L+1:nb_sample+3*L
    tp_sum = 0;
    for j=-L:L
        tp_sum = tp_sum + UPsymb_RX_filled(i-j)*g(j+L+1);
    end
    conv_sig_RX(i) = tp_sum;
end
% Vector cropping
% This part is simliar to the "Discrete Convolution" section of the function 
% "convolution_TX" and can therefore be directly copy-pasted (only variable names 
% need to be adjusted).
% Crop the resulting vector here
UPsymb_RX = conv_sig_RX(L+1:nb_sample+3*L);
%% Downsampling
% In order to retrieve the symbols at RX, the received filtered signal needs 
% to be sampled at the sampling rate (every _pT_ in slides 21-24 of the course 
% 2). This corresponds to downsample the filtered signal by a factor _sps_.
% Code the downsampling here
nb_symb = floor((nb_sample+2*L-1)/sps) +1;
symb_RX = zeros(nb_symb,1);
for i=1:nb_symb
    symb_RX(i) = UPsymb_RX((i-1)*sps + 1);
end
end
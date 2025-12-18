%[text] %[text:anchor:T_01D54C26] # Convolution (and downsampling)
%[text] This function implements a disrete convolution between a waveform and the impulse response of the RX filter. The resulting vector is then cropped and downsampled in order to retrieve the RX symbol vector.
%[text] <u>**Inputs:**</u>
%[text] - *signal*: signal vector of size *\[ ( nb\_symb - 1 ) \* sps + length( g ) \] x 1*
%[text] - *g*: Impulse Response (IR) of the pulse of size *1 x* length( *g* )
%[text] - *sps*: number of samples per symbol  \
%[text] <u>**Ouput:**</u>
%[text] - *symb\_RX*: filtered signal which corresponds to received symbols (column vector) \
function symb_RX = convolution_RX( signal , g , sps )
%[text:tableOfContents]{"heading":"**Table of Contents**"}
%[text] %[text:anchor:H_034D8590] ## Discrete Convolution
%[text] This part is simliar to the "Discrete Convolution" section of the function "convolution\_TX" and can therefore be directly copy-pasted (only variable names need to be adjusted)
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
%[text] %[text:anchor:H_91CC9298] ### Vector cropping
%[text] This part is simliar to the "Discrete Convolution" section of the function "convolution\_TX" and can therefore be directly copy-pasted (only variable names need to be adjusted).
% Crop the resulting vector here
UPsymb_RX = conv_sig_RX(L+1:nb_sample+3*L);
%[text] %[text:anchor:H_D7D82C77] ## Downsampling
%[text] In order to retrieve the symbols at RX, the received filtered signal needs to be sampled at the sampling rate (every *pT* in slides 21-24 of the course 2). This corresponds to downsample the filtered signal by a factor *sps*.
% Code the downsampling here
nb_symb = floor((nb_sample+2*L-1)/sps) +1;

symb_RX = zeros(nb_symb,1);
for i=1:nb_symb
    symb_RX(i) = UPsymb_RX((i-1)*sps + 1);
end

end

%[appendix]{"version":"1.0"}
%---

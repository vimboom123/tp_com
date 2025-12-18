%[text] %[text:anchor:T_027FEE12] # Generation of the channel model
%[text] This function implements the Rayleigh and Rice channel models. The details of such models can be found in the chapter 2 of the course of Philippe De Doncker (see especially section 2.2.2.).
%[text] <u>**Inputs:**</u>
%[text] - *nb\_realization*: indicates the number of realizations of the channel (scalar quantity)
%[text] - *channel\_type*: indicates the type of channel to generate (string quantity). Can take 2 values: *Rayleigh, Rice*
%[text] - *K:* Rice factor (scalar quantity) \
%[text]  <u>**Ouput:**</u>
%[text] - *channel*: channel response (vector of size *1 x nb\_realization*) \
function H = generate_channel( nb_realization , channel_type , K )
switch channel_type
%[text:tableOfContents]{"heading":"**Table of Contents**"}
%[text] %[text:anchor:H_0951CEE4] ## Rayleigh channel (*N*-waves model)
%[text] In a rich multipath Non-Line-Of-Sight (NLOS) environment, the propagation channel can be well modeled by a random variable following a complex normal distribution such as:
%[text] $h\_{Rayleigh}\\sim\\mathit{C} \\mathcal{N}(0,1)$ 
%[text] In this case, the channel magnitude $|h\_{Rayleigh}|$has a Rayleigh distribution and the channel phase $\\angle h\_{Rayleigh}$ has a uniform distribution over $2\\pi$.
%[text] In this code, the following normalization, $\\mathrm{E}\\left\[ |h\_{Rayleigh}|^2 \\right\]=1$, should be observed (i.e., the variance of both the real and imaginary parts of $h\_{Rayleigh}$ is $\\sigma^2=\\frac{1}{2}$).
    case 'Rayleigh'
        % Code the Rayleigh channel here
        H_Ray_mag = raylrnd(1,1,nb_realization);
        H_Ray_mag = H_Ray_mag / sqrt( (sum(H_Ray_mag.^2)/nb_realization) );

        H_Ray_ph = 2*pi*rand(1,nb_realization);

        H = H_Ray_mag .* exp(1i*H_Ray_ph);
%[text] %[text:anchor:H_EFEF96B5] ## Rice channel (*1+N*-waves model)
%[text] In a rich multipath environment, if one multipath components (MPC) dominates the other multipath, the propagation channel can be well modeled by:
%[text] $h\_{Rice}=\\sqrt{\\frac{K}{1+K}}h\_{strong MPC}+\\sqrt{\\frac{1}{1+K}}h\_{Rayleigh}$
%[text] where:
%[text] - $h\_{Rayleigh}\\sim\\mathit{C} \\mathcal{N}(0,1)$ with the following normalization $\\mathrm{E}\\left\[ |h\_{Rayleigh}|^2 \\right\]=1$ (i.e., the variance of both the real and imaginary parts of $h\_{Rayleigh}$ is $\\sigma^2=\\frac{1}{2}$).
%[text] - $h\_{strong MPC}=e^{j\\phi}$ is a deterministic magnitude and phase value of a strong MPC. Here the magnitude is 1 so that the amplitude of the strong MPC is completely characterized with the Rice factor K.
%[text] - $K=\\frac{ |h\_{strong MPC}|^2 }{\\mathrm{E}\\left\[ |h\_{NLOS}|^2 \\right\]}$ is the Rice factor
%[text] - $|h\_{Rice}|$has a Rice distribution. \
%[text] Typcally, the strong MPC can correspond to the Line-Of-Sight (LOS) contribution or to a strong reflection on a large wall for instance, whereas the second term $h\_{Rayleigh}$ corresponds to the NLOS contributions.
%[text] **Tip:** You can put any specific value for $\\phi$ and observe the influence on the results in the main.
    case 'Rice'
        % Code the Rice channel here
        H_Ray_mag = raylrnd(1,1,nb_realization);
        H_Ray_mag = H_Ray_mag / sqrt( (sum(H_Ray_mag.^2)/nb_realization) );

        H_Ray_ph = 2*pi*rand(1,nb_realization);
        H_Ray = H_Ray_mag .* exp(1i*H_Ray_ph);

        phy = 1;
        H_MPC = ones(1,nb_realization)*1*exp(1i*phy);
        
        H = sqrt(K/(1+K)).*H_MPC + sqrt(1/(1+K)).*H_Ray;
end

%[appendix]{"version":"1.0"}
%---

function H = generate_channel( nb_realization , channel_type , K )
%% GENERATE_CHANNEL Generation of the channel model
% This function implements the Rayleigh and Rice channel models. The details 
% of such models can be found in the chapter 2 of the course of Philippe De Doncker 
% (see especially section 2.2.2.).
% 
% *Inputs:*
%% 
% * _nb_realization_: indicates the number of realizations of the channel (scalar 
% quantity)
% * _channel_type_: indicates the type of channel to generate (string quantity). 
% Can take 2 values: _Rayleigh, Rice_
% * _K:_ Rice factor (scalar quantity)
%% 
% *Ouput:*
%% 
% * _channel_: channel response (vector of size _1 x nb_realization_)
switch channel_type
%% Rayleigh channel (_N_-waves model)
% In a rich multipath Non-Line-Of-Sight (NLOS) environment, the propagation 
% channel can be well modeled by a random variable following a complex normal 
% distribution such as:
% 
% $$h_{Rayleigh}\sim\mathit{C} \mathcal{N}(0,1)$$ 
% 
% In this case, the channel magnitude $|h_{Rayleigh}|$has a Rayleigh distribution 
% and the channel phase $\angle h_{Rayleigh}$ has a uniform distribution over 
% $2\pi$.
% 
% In this code, the following normalization, $\mathrm{E}\left[ |h_{Rayleigh}|^2 
% \right]=1$, should be observed (i.e., the variance of both the real and imaginary 
% parts of $h_{Rayleigh}$ is $\sigma^2=\frac{1}{2}$).
    case 'Rayleigh'
        % Code the Rayleigh channel here
        H_Ray_mag = raylrnd(1,1,nb_realization);
        H_Ray_mag = H_Ray_mag / sqrt( (sum(H_Ray_mag.^2)/nb_realization) );
        H_Ray_ph = 2*pi*rand(1,nb_realization);
        H = H_Ray_mag .* exp(1i*H_Ray_ph);
%% Rice channel (_1+N_-waves model)
% In a rich multipath environment, if one multipath components (MPC) dominates 
% the other multipath, the propagation channel can be well modeled by:
% 
% $$h_{Rice}=\sqrt{\frac{K}{1+K}}h_{strong MPC}+\sqrt{\frac{1}{1+K}}h_{Rayleigh}$$
% 
% where:
%% 
% * $h_{Rayleigh}\sim\mathit{C} \mathcal{N}(0,1)$ with the following normalization 
% $\mathrm{E}\left[ |h_{Rayleigh}|^2 \right]=1$ (i.e., the variance of both the 
% real and imaginary parts of $h_{Rayleigh}$ is $\sigma^2=\frac{1}{2}$).
% * $h_{strong MPC}=e^{j\phi}$ is a deterministic magnitude and phase value 
% of a strong MPC. Here the magnitude is 1 so that the amplitude of the strong 
% MPC is completely characterized with the Rice factor K.
% * $K=\frac{ |h_{strong MPC}|^2 }{\mathrm{E}\left[ |h_{NLOS}|^2 \right]}$ is 
% the Rice factor
% * $|h_{Rice}|$has a Rice distribution.
%% 
% Typcally, the strong MPC can correspond to the Line-Of-Sight (LOS) contribution 
% or to a strong reflection on a large wall for instance, whereas the second term 
% $h_{Rayleigh}$ corresponds to the NLOS contributions.
% 
% *Tip:* You can put any specific value for $\phi$ and observe the influence 
% on the results in the main.
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
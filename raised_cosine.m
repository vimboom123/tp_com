function g = raised_cosine( rolloff , span , sps , shape )
%% RAISED_COSINE (Root) Raised Cosine Filter
% This function designs the Impulse Response (i.e., discrete-time pulse shape) 
% of a Raised Cosine (RC) filter and Root Raised Cosine (RRC) filter, typically 
% used for pulse shaping.
% 
% *Inputs:*
%% 
% * _rolloff_: rolloff factor of the raised cosine filter ($0\le \text{rolloff}\le 
% 1  $)
% * _span_: pulse impulse response duration in terms of symbols
% * _sps_: number of samples per symbol
% * _shape_: select Raised Cosine filter (_shape_ = 'normal') or Root Raised 
% Cosine filter (_shape_ = 'sqrt')
%% 
% *Ouput:*
%% 
% * _g_: Impulse Response (IR) of the pulse of size _1 x ( span . sps + 1 )_
    nb_sample_per_pulse = span * sps ;
    if rem( nb_sample_per_pulse , 2) ~= 0
        error('The product of sps and span must be even')
    end
    eps = 1e-3;
%% Guidelines for the pulse IR design
% General expressions
% The pulse shape of a *raised cosine* spectrum is given by the following relationship:
% 
% $$g_{RC}\left(t\right)=\text{sinc}\left(\pi t / T\right)\frac{\cos\left( \pi 
% \beta t / T\right)}{1-4 \beta^2 t^2 / T^2}$$
% 
% The pulse shape of a *root raised cosine* spectrum is given by the following 
% relationship:
% 
% $$g_{RRC}\left(t\right)=\frac{ 4\beta\cos\left[ \left( 1+\beta \right)\pi 
% t / T  \right]+\frac{\sin\left[ \left( 1-\beta \right)\pi t / T  \right]} {\left( 
% t/T\right)}}{ \pi*\sqrt{T} * \left[ 1 - 16 \beta^2t^2/T^2\right]}$$ 
% 
% where $\beta$ is the roll-off factor and $T$ is the symbol duration.
% Time truncation
% Both $g_{RC}\left(t\right)$and $g_{RRC}\left(t\right)$expressions lead to 
% Infinite Impusle Response (IIR) filters. However, since these pulses decay with 
% time, it is possible to truncate their Impulse Response (IR) without deteriorating 
% too much their spectrum and hence, their Nyquist criterion properties. One obtain 
% therefore Finite Impulse Response (FIR) filters which are more stable (but typically 
% more demanding than IIR filters in terms of computational resources).
% 
% We define the $ \text{span}$ of the filter as a parameter to truncate the 
% filter impulse response. The higher the $ \text{span}$, the longer the pulse 
% duration. $ \text{span}$ is expressed in terms of symbols. It is the number 
% of symbols during which the pulse IR lasts. The total pulse duration is therefore 
% equal to $ \text{span}.T $, where $ T $ is the symbol duration. For instance, 
% if $ \text{span}=5$, the pulse impulse response lasts during $ 5T$: 2 symbols 
% before and 2 symbols after the symbol $s_k$ under consideration. The figure 
% below illustrates 2 examples of $ \text{span}$ values used to truncate the pulse.
% 
% 
% 
% Higher $ \text{span}$ values better approximate the original IIR but requires 
% more computational resources, so a tradeoff is to be chosen. Typical values 
% of impulse response duration are 8-16 symbols. High values of $ \text{span}$ 
% potentially leads to lower ISI at the receiver side. High values are also necessary 
% to reach a certain level of out-of-band attenuation (when the impulse response 
% is truncated in time, the out-ot-band spectrum is no longer perfeclty zero, 
% and exhibits pass-band ripples, a certain slope, and out-of-band sidelobes).
% Discrete time
% The expressions given for the pulse shape are in continuous time. The first 
% step is therefore to discritize the time vector such as:
% 
% $t=nT_S$, with $T_S$ the sampling time related to the sampling frequency $f_S$ 
% by $T_S=1/f_S$.
% 
% By including the truncation, the discrete-time vector can be expressed as:
% 
% $$t=-\frac{\text{span}}{2}T:T_S:\frac{\text{span}}{2}T$$
% 
% The variable _sps_ sets the number of samples per symbol. Thanks to this parametres, 
% oversampling ratio can be easily adjusted. So the symbol duration can be expressed 
% as:
% 
% $$T=\text{sps}.T_S$$
% 
% Finally, the time domain expressions of the pulses can be expressed with no 
% other parameters than _span_ and _sps_ in addition to the _rolloff_ factor value 
% by observing that:
% 
% $$t/T=-\frac{\text{span}}{2}:\frac{1}{\text{sps}}:\frac{\text{span}}{2}$$
% 
% The pulse impulse response must contain an odd number of samples equal to 
% _sps.span + 1_. So the product _sps.span_ should be even.
% 
% *Note for RCC filter!* Even after replacing the vector $t/T$ by the above 
% expression in $g_{RRC}\left(t\right)$, a $\sqrt{T}$ term remains. This constant 
% term can however be simply dropped (e.g., assuming $T=1$) since it only affects 
% the filter amplitude and does not affect its shape. The filter amplitude will 
% be anyway normalized such as to exhibit a unit energy.
% 
% *Be carefull!* The sinc function in Matlab does automatically introduce $\pi$in 
% its argument (see the help of the _sinc_ function).
%% Raised Cosine (RC) filter
% When $|t/T|\to \frac{1}{2\beta}$, the $g_{RC}\left(t\right)$expression assumes 
% an indeterminate form. The Matlab result will therefore not be accurate. It 
% is then required to anaytically calculate the limit $\lim_{|t/T| \to \frac{1}{2\beta}} 
% g_{RC}(t)$ and to use the resulting expression to calculate $g_{RC}\left(t\right)$when 
% $|t/T|= \frac{1}{2\beta}$. To calculate this limit, you can use L'Hôpital's 
% rule for the term $\frac{\cos\left( \pi \beta t / T\right)}{1-4 \beta^2 t^2 
% / T^2}$ in $g_{RC}\left(t\right)$.
% 
% *Tips*: At first, choose a rolloff factor $\beta$ value that does not involve 
% $|t/T|=\frac{1}{2\beta}$. In that case, the indeterminate form issue will not 
% raise and you will be able to code the RC filter and to test whether your implementation 
% is correct or not. Once it works, choose a rolloff factor $\beta$ value for 
% which the condition $|t/T|=\frac{1}{2\beta}$ does occur. You will have to calculate 
% manually the limit $\lim_{|t/T| \to \frac{1}{2\beta}} g_{RC}(t)$ and to implement 
% it in order to make your filter impulse response correct again.
    if strncmp(shape, 'normal', 1)
    % Code here the raised cosine (RC) impulse response (output: g)
    beta = rolloff;
    t_T = -span/2 : 1/sps : +span/2;
    g = zeros(1,span*sps+1);
    for i=1:span*sps+1
        if ( abs(abs(t_T(i)) - 1/(2*beta)) <= eps )
            g(i) = sinc(1/(2*beta))*pi/4;
        else
            g(i) = sinc(t_T(i))*cos(pi*beta*t_T(i))/(1-4*t_T(i)^2 * beta^2);
        end
    end
%% Root Raised Cosine (RRC) filter
% Unlike the raised-cosine filter, the RRC impulse response is not zero at the 
% intervals of ±_Ts_. However, the combined transmit and receive filters form 
% a raised-cosine filter which does have zero at the intervals of ±_Ts_. Only 
% in the case of _β_=0 does the root raised-cosine have zeros at ±_Ts_.
% 
% The expression $g_{RRC}\left(t\right)$ assumes an indeterminate form when 
% the vector $t/T$ has the following values:
%% 
% * $t/T=0$ (use $\sin x \simeq  x$ to calculate the limit)
% * $|t/T|=\frac{1}{4\beta}$ (use L'Hôpital's rule to calculate the limit)
%% 
% Use the expressions found for those limits to implement the calculation of 
% these specific points in Matlab.
% 
% *Tips*: Similarly to the implementation of the RC filter, choose a rolloff 
% factor $\beta$ value that does not involve $|t/T|=\frac{1}{4\beta}$ to avoid 
% dealing with this indeterminate form in a first approach. However, you cannot 
% avoid the indeterminate form when $t/T=0$ and should deal with it whatever the 
% rolloff factor $\beta$ value (but this limit is straightforward to calculate).
    elseif strncmp(shape, 'sqrt', 1)
    % Code here the root raised cosine (RRC) impulse response (output: g)
    beta = rolloff;
    t_T = -span/2 : 1/sps : +span/2;
    g = zeros(1,span*sps+1);
    for i=1:span*sps+1
        if ( abs(abs(t_T(i)) - 0) <= eps )
            g(i) = (4*beta +  (1-beta)*pi) / pi;
        
        elseif ( abs(abs(t_T(i)) - 1/(4*beta)) <= eps )
            g(i) = (1+beta)*sin((1+beta)*pi/(4*beta))/2 - (1-beta)*cos((1-beta)*pi/(4*beta))/2 + 2*beta*sin((1-beta)*pi/(4*beta))/pi;
        
        else
            g(i) = (4*beta*cos((1+beta)*pi*t_T(i)) + sin((1-beta)*pi*t_T(i))/t_T(i)) / (pi*(1-16*beta^2 *t_T(i)^2));
        end
    end
    
    end
%% Filter Energy Normalization 
% The filter amplitude is normalized in order to exhibit a unit energy. With 
% this normalization, the amplitude of a TX symbol will remain constant after 
% passing successively through TX and RX filter (if the propagation channel is 
% not considered, i.e., $h=1$).
    g = g / sqrt( sum( g.^2 ) ) ;                                   
end
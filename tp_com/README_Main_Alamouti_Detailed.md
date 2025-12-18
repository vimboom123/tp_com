# Documentation Détaillée: Main_Alamouti_Full_Project.m

## 📖 Introduction

Ce document fournit une explication **ligne par ligne** du code `Main_Alamouti_Full_Project.m`, en établissant des liens directs avec les formules théoriques des **Slides 24-25** du cours "Unit 4 - 3 - MIMO Processing".

---

## 🎯 Objectif du Code

Implémenter une simulation complète du schéma **Alamouti STBC (Space-Time Block Code)** pour un système **MISO 2×1** (2 antennes TX, 1 antenne RX), et comparer ses performances BER avec un système **SISO** de référence (main5).

---

## 📐 Rappel Théorique (Slides 24-25)

### Schéma Alamouti

Le code Alamouti est un **code espace-temps en bloc** qui permet d'obtenir une **diversité d'ordre 2** avec seulement **2 antennes d'émission** et **1 antenne de réception**.

### Matrice de Transmission (Slide 24)

Pour deux symboles consécutifs $s_1$ et $s_2$, la matrice de transmission est:

$$
\mathbf{X} = \begin{bmatrix} x_{11} & x_{12} \\ x_{21} & x_{22} \end{bmatrix} = \frac{1}{\sqrt{2}} \begin{bmatrix} s_1 & -s_2^* \\ s_2 & s_1^* \end{bmatrix}
$$

où:
- Ligne 1 = Antenne TX1
- Ligne 2 = Antenne TX2
- Colonne 1 = Intervalle de temps $t_1$
- Colonne 2 = Intervalle de temps $t_2$

### Signaux Reçus (Slide 25)

À l'antenne RX unique:

$$
y_1 = h_1 x_{11} + h_2 x_{21} + n_1 = \frac{1}{\sqrt{2}}(h_1 s_1 + h_2 s_2) + n_1
$$

$$
y_2 = h_1 x_{12} + h_2 x_{22} + n_2 = \frac{1}{\sqrt{2}}(-h_1 s_2^* + h_2 s_1^*) + n_2
$$

### Décodage Alamouti (Slide 25)

Le décodeur calcule:

$$
z_1 = \frac{1}{\sqrt{2}}(h_1^* y_1 + h_2 y_2^*)
$$

$$
z_2 = \frac{1}{\sqrt{2}}(h_2^* y_1 - h_1 y_2^*)
$$

Après substitution et simplification:

$$
z_1 = \frac{|h_1|^2 + |h_2|^2}{2} s_1 + \tilde{n}_1
$$

$$
z_2 = \frac{|h_1|^2 + |h_2|^2}{2} s_2 + \tilde{n}_2
$$

Où $\alpha = \frac{|h_1|^2 + |h_2|^2}{2}$ est le **gain de diversité**.

### Estimation Finale

$$
\hat{s}_1 = \frac{z_1}{\alpha}, \quad \hat{s}_2 = \frac{z_2}{\alpha}
$$

---

## 📝 Explication Ligne par Ligne

### Lignes 1-4: En-tête et Initialisation

```matlab
%% Projet 3: Simulation complète Alamouti MISO (aligné avec main5)
% Basé sur main_5, utilisant les mêmes paramètres : 16-QAM + estimation par pilotes
% Conforme aux formules standard de décodage Alamouti (Slides 24-25)
clc; clear all; close all;
```

**Explication:**
- `clc`: Efface la fenêtre de commande
- `clear all`: Supprime toutes les variables de l'espace de travail
- `close all`: Ferme toutes les fenêtres de figures

**Lien théorique:** Aucun - initialisation standard MATLAB.

---

### Lignes 6-15: Paramètres Globaux

```matlab
%% === Paramètres globaux (identiques à main5) ===
nb_data = 1000;             % Nombre de symboles de données par trame
nb_pilot = 10;              % Nombre de symboles pilotes
nb_bit_per_symb = 4;        % 16-QAM
rolloff = 0.5;
symb_rate = 100e6;
sps = 2;
span = 16;
fs = symb_rate * sps;
nb_bit = nb_data * nb_bit_per_symb;
```

| Variable | Valeur | Description |
|----------|--------|-------------|
| `nb_data` | 1000 | Nombre de symboles par trame (doit être **pair** pour Alamouti) |
| `nb_pilot` | 10 | Symboles pilotes pour estimation du canal |
| `nb_bit_per_symb` | 4 | $\log_2(M) = \log_2(16) = 4$ bits/symbole (16-QAM) |
| `rolloff` | 0.5 | Facteur de roll-off du filtre RRC: $\beta = 0.5$ |
| `symb_rate` | 100 MHz | Débit symbole $R_s$ |
| `sps` | 2 | Échantillons par symbole (suréchantillonnage) |
| `span` | 16 | Durée du filtre en symboles |
| `fs` | 200 MHz | Fréquence d'échantillonnage: $f_s = R_s \times \text{sps}$ |
| `nb_bit` | 4000 | Total bits par trame: $1000 \times 4$ |

**Lien théorique:** Ces paramètres définissent le système de communication. Le choix `nb_data = 1000` (pair) est **essentiel** car Alamouti traite les symboles par paires $(s_1, s_2)$.

---

### Lignes 17-22: Conception du Filtre et Pilotes

```matlab
% Conception du filtre
g = raised_cosine(rolloff, span, sps, 'sqrt');

% Génération des symboles pilotes (identique à main5)
bit_pilot = randi([0 1], nb_pilot * nb_bit_per_symb, 1);
symb_pilot = mapping_QAM(bit_pilot, nb_bit_per_symb, length(bit_pilot));
```

**Ligne 18:** Crée le filtre **Root Raised Cosine (RRC)** avec:
- Réponse impulsionnelle de longueur `span × sps + 1 = 33` échantillons
- Le facteur `'sqrt'` indique la racine carrée (pour cascade TX-RX)

**Lignes 21-22:** Génère des pilotes aléatoires 16-QAM:
- `randi([0 1], 40, 1)`: 40 bits aléatoires
- `mapping_QAM`: Convertit en 10 symboles 16-QAM

**Lien théorique:** Les pilotes sont utilisés pour l'**estimation du canal** par la méthode **Least Squares (LS)**:

$$
\hat{h} = \frac{1}{N_p} \sum_{i=1}^{N_p} \frac{y_{\text{pilot},i}}{p_i}
$$

---

### Lignes 27-34: Génération des Données TX

```matlab
fprintf('Exécution Partie A: Visualisation (SNR = 20dB, 16-QAM)...\n');

vis_snr_dB = 20; 
vis_snr_lin = 10^(vis_snr_dB/10);

% Génération d'une trame de données
bit_TX = randi([0 1], nb_bit, 1);
symb_TX_data = mapping_QAM(bit_TX, nb_bit_per_symb, nb_bit);
```

**Ligne 29-30:** Conversion SNR de dB vers linéaire:

$$
\text{SNR}_{\text{lin}} = 10^{\text{SNR}_{\text{dB}}/10} = 10^{20/10} = 100
$$

**Lignes 33-34:** Génère 4000 bits aléatoires → 1000 symboles 16-QAM.

---

### Lignes 36-53: ⭐ Codage Alamouti (CRUCIAL)

```matlab
% --- Codage Alamouti (Slide 24) ---
% Appariement des symboles: (s1, s2), (s3, s4), ...
s1 = symb_TX_data(1:2:end);  % Symboles aux positions impaires
s2 = symb_TX_data(2:2:end);  % Symboles aux positions paires
nb_pairs = length(s1);

% Normalisation de puissance: chaque antenne émet 1/sqrt(2) fois
scale = 1/sqrt(2);

% Construction des séquences pour les deux antennes
symb_TX1 = zeros(nb_data, 1); 
symb_TX2 = zeros(nb_data, 1);
symb_TX1(1:2:end) = s1 * scale; 
symb_TX2(1:2:end) = s2 * scale;
symb_TX1(2:2:end) = -conj(s2) * scale;
symb_TX2(2:2:end) = conj(s1) * scale;
```

#### Décomposition Détaillée:

**Lignes 38-39:** Séparation des symboles en paires:
- `s1 = [symb(1), symb(3), symb(5), ...]` → 500 symboles
- `s2 = [symb(2), symb(4), symb(6), ...]` → 500 symboles

**Ligne 43:** Facteur de normalisation de puissance:

$$
\text{scale} = \frac{1}{\sqrt{2}}
$$

**Pourquoi $\frac{1}{\sqrt{2}}$?** Pour que la puissance totale transmise reste égale à 1:

$$
P_{\text{total}} = |x_{11}|^2 + |x_{21}|^2 = \left|\frac{s_1}{\sqrt{2}}\right|^2 + \left|\frac{s_2}{\sqrt{2}}\right|^2 = \frac{|s_1|^2 + |s_2|^2}{2}
$$

**Lignes 48-53:** Construction de la matrice Alamouti:

| Indice temps | TX1 (`symb_TX1`) | TX2 (`symb_TX2`) |
|--------------|------------------|------------------|
| 1, 3, 5, ... (impair) | $\frac{s_k}{\sqrt{2}}$ | $\frac{s_{k+1}}{\sqrt{2}}$ |
| 2, 4, 6, ... (pair) | $\frac{-s_{k+1}^*}{\sqrt{2}}$ | $\frac{s_k^*}{\sqrt{2}}$ |

**Correspondance avec Slide 24:**

$$
\text{Ligne 50: } x_{11} = \frac{s_1}{\sqrt{2}} \quad \text{(TX1, temps 1)}
$$

$$
\text{Ligne 51: } x_{21} = \frac{s_2}{\sqrt{2}} \quad \text{(TX2, temps 1)}
$$

$$
\text{Ligne 52: } x_{12} = \frac{-s_2^*}{\sqrt{2}} \quad \text{(TX1, temps 2)}
$$

$$
\text{Ligne 53: } x_{22} = \frac{s_1^*}{\sqrt{2}} \quad \text{(TX2, temps 2)}
$$

---

### Lignes 55-66: Pilotes Orthogonaux pour Alamouti

```matlab
% --- Ajout des pilotes ---
pilot_ala = symb_pilot(1:min(nb_pilot, length(symb_pilot)));
nb_pilot_ala = length(pilot_ala);

% Utilisation de 2 intervalles pilotes orthogonaux
pilot_TX1 = [pilot_ala; zeros(nb_pilot_ala, 1)];  % TX1: pilot, 0
pilot_TX2 = [zeros(nb_pilot_ala, 1); pilot_ala];  % TX2: 0, pilot

% Trame complète: [Pilotes | Données]
frame_TX1 = [pilot_TX1; symb_TX1];
frame_TX2 = [pilot_TX2; symb_TX2];
```

**Principe des pilotes orthogonaux:**

| Intervalle | TX1 | TX2 | Signal reçu |
|------------|-----|-----|-------------|
| 1 à 10 | pilot | 0 | $y = h_1 \cdot \text{pilot}$ → Estime $h_1$ |
| 11 à 20 | 0 | pilot | $y = h_2 \cdot \text{pilot}$ → Estime $h_2$ |

**Lien théorique:** Cette conception orthogonale permet d'estimer **séparément** $h_1$ et $h_2$:

$$
\hat{h}_1 = \frac{1}{N_p} \sum_{i=1}^{N_p} \frac{y_i^{(1)}}{p_i}, \quad \hat{h}_2 = \frac{1}{N_p} \sum_{i=1}^{N_p} \frac{y_i^{(2)}}{p_i}
$$

---

### Lignes 68-77: Mise en Forme d'Impulsion et Spectre

```matlab
% --- Mise en forme d'impulsion ---
sig_TX1 = convolution_TX(frame_TX1, g, sps);
sig_TX2 = convolution_TX(frame_TX2, g, sps);

% --- Figure 1: Spectre ---
figure('Name', 'Spectre TX');
f = linspace(-fs/2, fs/2, length(sig_TX1));
plot(f*1e-6, 10*log10(abs(fftshift(fft(sig_TX1)))));
grid on; title('Spectre Antenne TX 1 (Alamouti, 16-QAM)');
xlabel('Fréquence [MHz]'); ylabel('Amplitude [dB]');
```

**`convolution_TX`:** 
1. Suréchantillonnage par `sps` (insertion de zéros)
2. Convolution avec le filtre RRC `g`

**Bande passante résultante:**

$$
B = (1 + \beta) \times R_s = (1 + 0.5) \times 100 = 150 \text{ MHz}
$$

---

### Lignes 79-90: Canal de Rayleigh et Bruit

```matlab
% --- Canal de Rayleigh ---
H1_vis = (randn + 1i*randn)/sqrt(2);
H2_vis = (randn + 1i*randn)/sqrt(2);

% Signal reçu = h1*x1 + h2*x2
sig_RX_noiseless = H1_vis * sig_TX1 + H2_vis * sig_TX2;

% Ajout de bruit
rx_pwr = sum(abs(sig_TX1).^2 + abs(sig_TX2).^2)/length(sig_TX1)*sps;
noise_var = rx_pwr / vis_snr_lin;
noise = sqrt(noise_var/2)*(randn(size(sig_RX_noiseless)) + 1i*randn(size(sig_RX_noiseless)));
sig_RX = sig_RX_noiseless + noise;
```

**Lignes 80-81:** Génération du canal Rayleigh:

$$
h \sim \mathcal{CN}(0, 1) = \frac{\mathcal{N}(0,1) + j\mathcal{N}(0,1)}{\sqrt{2}}
$$

Normalisation: $\mathbb{E}[|h|^2] = 1$

**Ligne 84:** Modèle de réception MISO (Slide 25):

$$
y = h_1 x_1 + h_2 x_2 + n
$$

**Lignes 87-90:** Calcul du bruit AWGN:
- `rx_pwr`: Puissance totale transmise
- `noise_var = rx_pwr / SNR`: Variance du bruit
- Division par 2 car bruit complexe (parties réelle et imaginaire)

---

### Lignes 92-106: Filtrage RX et Estimation du Canal

```matlab
% --- Filtrage de réception ---
frame_len = 2*nb_pilot_ala + nb_data;
symb_RX_raw = convolution_RX(sig_RX, g, sps);
symb_RX_raw = symb_RX_raw(span+1:span+frame_len);

% --- Estimation du canal par pilotes ---
pilot_RX1 = symb_RX_raw(1:nb_pilot_ala);
pilot_RX2 = symb_RX_raw(nb_pilot_ala+1:2*nb_pilot_ala);

H1_est = sum(pilot_RX1 ./ pilot_ala) / nb_pilot_ala;
H2_est = sum(pilot_RX2 ./ pilot_ala) / nb_pilot_ala;
```

**`convolution_RX`:**
1. Convolution avec le filtre RRC (filtre adapté)
2. Sous-échantillonnage par `sps`

**Ligne 95:** Suppression du délai du filtre (`span` symboles).

**Lignes 99-103:** Estimation LS des canaux:

$$
\hat{h}_1 = \frac{1}{N_p} \sum_{i=1}^{N_p} \frac{y_i^{(\text{pilot1})}}{p_i}
$$

$$
\hat{h}_2 = \frac{1}{N_p} \sum_{i=1}^{N_p} \frac{y_i^{(\text{pilot2})}}{p_i}
$$

---

### Lignes 124-143: ⭐ Décodage Alamouti (CRUCIAL)

```matlab
% --- Décodage Alamouti (Formule standard Slide 25) ---
y1 = symb_RX_data(1:2:end);   % Réception intervalles impairs
y2 = symb_RX_data(2:2:end);   % Réception intervalles pairs

% Formule de décodage standard
y2c = conj(y2);
z1 = (1/sqrt(2)) * (conj(H1_est).*y1 + H2_est.*y2c);
z2 = (1/sqrt(2)) * (conj(H2_est).*y1 - H1_est.*y2c);

% Égalisation: division par alpha
alpha = (abs(H1_est)^2 + abs(H2_est)^2) / 2;
x1_est = z1 / alpha;
x2_est = z2 / alpha;

% Reconstruction
symb_est = zeros(nb_data, 1);
symb_est(1:2:end) = x1_est;
symb_est(2:2:end) = x2_est;
```

#### Dérivation Mathématique Complète (Slide 25):

**Étape 1:** Signaux reçus

$$
y_1 = \frac{1}{\sqrt{2}}(h_1 s_1 + h_2 s_2) + n_1
$$

$$
y_2 = \frac{1}{\sqrt{2}}(-h_1 s_2^* + h_2 s_1^*) + n_2
$$

**Étape 2:** Calcul de $z_1$ (Ligne 132)

$$
z_1 = \frac{1}{\sqrt{2}}(h_1^* y_1 + h_2 y_2^*)
$$

Substituons:

$$
z_1 = \frac{1}{\sqrt{2}} \left[ h_1^* \cdot \frac{1}{\sqrt{2}}(h_1 s_1 + h_2 s_2) + h_2 \cdot \frac{1}{\sqrt{2}}(-h_1^* s_2 + h_2^* s_1) \right]
$$

$$
z_1 = \frac{1}{2} \left[ |h_1|^2 s_1 + h_1^* h_2 s_2 - h_1^* h_2 s_2 + |h_2|^2 s_1 \right]
$$

$$
z_1 = \frac{|h_1|^2 + |h_2|^2}{2} s_1 + \tilde{n}_1
$$

**Étape 3:** Calcul de $z_2$ (Ligne 133)

$$
z_2 = \frac{1}{\sqrt{2}}(h_2^* y_1 - h_1 y_2^*)
$$

Par un calcul similaire:

$$
z_2 = \frac{|h_1|^2 + |h_2|^2}{2} s_2 + \tilde{n}_2
$$

**Étape 4:** Égalisation (Lignes 136-138)

$$
\alpha = \frac{|h_1|^2 + |h_2|^2}{2}
$$

$$
\hat{s}_1 = \frac{z_1}{\alpha} = s_1 + \frac{\tilde{n}_1}{\alpha}
$$

$$
\hat{s}_2 = \frac{z_2}{\alpha} = s_2 + \frac{\tilde{n}_2}{\alpha}
$$

**Propriété clé:** Le gain de diversité $\alpha = \frac{|h_1|^2 + |h_2|^2}{2}$ combine les deux canaux, ce qui explique l'**ordre de diversité 2**.

---

### Lignes 159-272: Simulation Monte Carlo BER

```matlab
nb_frame_ber = 500;
Lsnr_dB = 0:2:20;
```

La boucle principale simule 500 trames pour chaque valeur de SNR.

#### Structure SISO (Lignes 179-202):
- Identique à main5
- Estimation canal: $\hat{h} = \frac{1}{N_p} \sum \frac{y_i}{p_i}$
- Égalisation ZF: $\hat{s} = y / \hat{h}$

#### Structure Alamouti (Lignes 204-260):
- Codage Alamouti (Lignes 206-210)
- Pilotes orthogonaux (Lignes 214-215)
- Décodage standard (Lignes 249-254)

---

### Lignes 274-295: Courbes BER et Théorie

```matlab
% Courbes théoriques
EbNo_dB = Lsnr_dB - 10*log10(nb_bit_per_symb);
M = 2^nb_bit_per_symb;  % 16-QAM

ber_theory_siso = berfading(EbNo_dB, 'qam', M, 1);
ber_theory_ala = berfading(EbNo_dB, 'qam', M, 2);
```

**Ligne 281:** Conversion SNR → Eb/No:

$$
\frac{E_b}{N_0} (\text{dB}) = \text{SNR} (\text{dB}) - 10 \log_{10}(\log_2 M)
$$

$$
\frac{E_b}{N_0} = \text{SNR} - 10 \log_{10}(4) = \text{SNR} - 6 \text{ dB}
$$

**Lignes 285-286:** BER théorique Rayleigh:
- `berfading(..., 1)`: Ordre de diversité 1 (SISO)
- `berfading(..., 2)`: Ordre de diversité 2 (Alamouti)

---

## 📊 Résumé des Correspondances Code ↔ Théorie

| Ligne(s) | Code | Formule Théorique (Slide) |
|----------|------|---------------------------|
| 43 | `scale = 1/sqrt(2)` | $\frac{1}{\sqrt{2}}$ (normalisation puissance) |
| 50 | `symb_TX1(1:2:end) = s1 * scale` | $x_{11} = \frac{s_1}{\sqrt{2}}$ |
| 51 | `symb_TX2(1:2:end) = s2 * scale` | $x_{21} = \frac{s_2}{\sqrt{2}}$ |
| 52 | `symb_TX1(2:2:end) = -conj(s2) * scale` | $x_{12} = \frac{-s_2^*}{\sqrt{2}}$ |
| 53 | `symb_TX2(2:2:end) = conj(s1) * scale` | $x_{22} = \frac{s_1^*}{\sqrt{2}}$ |
| 84 | `H1_vis * sig_TX1 + H2_vis * sig_TX2` | $y = h_1 x_1 + h_2 x_2 + n$ |
| 132 | `z1 = (1/sqrt(2)) * (conj(H1_est).*y1 + H2_est.*y2c)` | $z_1 = \frac{1}{\sqrt{2}}(h_1^* y_1 + h_2 y_2^*)$ |
| 133 | `z2 = (1/sqrt(2)) * (conj(H2_est).*y1 - H1_est.*y2c)` | $z_2 = \frac{1}{\sqrt{2}}(h_2^* y_1 - h_1 y_2^*)$ |
| 136 | `alpha = (abs(H1_est)^2 + abs(H2_est)^2) / 2` | $\alpha = \frac{|h_1|^2 + |h_2|^2}{2}$ |
| 137-138 | `x1_est = z1 / alpha` | $\hat{s}_1 = \frac{z_1}{\alpha}$ |

---

## 🔑 Points Clés à Retenir

1. **Normalisation $\frac{1}{\sqrt{2}}$**: Garantit que la puissance totale reste constante.

2. **Pilotes orthogonaux**: Permettent d'estimer séparément $h_1$ et $h_2$.

3. **Décodage linéaire**: Le décodeur Alamouti est un simple **combineur linéaire**.

4. **Gain de diversité**: Le terme $\alpha = \frac{|h_1|^2 + |h_2|^2}{2}$ combine les deux canaux.

5. **Ordre de diversité 2**: La pente BER est proportionnelle à $\frac{1}{\text{SNR}^2}$ au lieu de $\frac{1}{\text{SNR}}$ pour SISO.

---

## 📚 Références

- **Slide 24**: Matrice de transmission Alamouti
- **Slide 25**: Formules de décodage et analyse de performance
- **Alamouti, S.M. (1998)**: "A simple transmit diversity technique for wireless communications"


---
DC.date: "2020-11-09"
DC.source: 'radar\_target\_generation\_and\_detection.m'
Generator: "MATLAB R2020b"
Title: 'radar\_target\_generation\_and\_detection'
Author: Muhammad Khaled
---


## Implementation steps for the 2D CFAR process
1. We select the number of training cells for each dimension Tr and Td. Similarly, pick the number of guard cells Gr and Gd.
2. We need to slide the Cell Under Test (CUT) across the complete cell matrix. In order to do that, we specify the part of the grid 
that includes the training, guard and test cells for the current CUT. The sub grid size can be calculated by: (2Tr+2Gr+1)(2Td+2Gd+1)
3. The total number of cells in the guard region and cell under test: (2Gr+1)(2Gd+1)
This gives the Training Cells : (2Tr+2Gr+1)(2Td+2Gd+1) - (2Gr+1)(2Gd+1)
4. We measure and average the noise across all the training cells. This gives us the threshold.
5. We add the offset (if in signal strength in dB) to the threshold to keep the false alarm to the minimum.
6. We check the signal level at the Cell Under Test. If the CUT signal level is greater than the Threshold, assign a value of 1, else equate it to zero.
7. Since the cell under test are not located at the edges, due to the training cells occupying the edges, we suppress the edges to zero.


## Selection of Training, Guard cells and offset:
After several iteration, I found that by increasing the number of training cells, the results gets better. My intuition is that: The number of training cells should be decided based on the environment. If a dense traffic scenario then the fewer training cells should be used, as closely spaced targets can impact the noise estimate. In our case, we have only one target so more training cells will help us have more accurate threshold value.

I also noticed how important is the offset, as I set it to values <= 1, the results are too noisy and the target can be hardly identified.

The number of guard cells should be decided based on the leakage of the target signal out of the cell under test. If target reflections are strong they often get into surrounding bins. As we have only one target, the signal reflected from it is quiet strong. Although as small number of guard cells gave good results, I preferred to keep the 4 guard cells to prevent the reflected signal from affecting the neighboring cells.

## Steps taken to suppress the non-thresholded cells at the edges:
I chose to modify the RDM in the same variable by iterating through the whole map and checking for the strength of the reflected signal. The CUT which had value above the threshold + offset is set to 1, otherwise it is set to 0. After finishing, we would not have iterated over the cells at the edges, and these cells would have values which are neither 1 or 0 ( noise will not let that happen :D ). So I wrote a condition that checks if the cell value is neither 0 or 1 and if it is satisfied, I set the cell to 0.

<div class="content">

## Contents

<div>

- [Radar Specifications]
- [User Defined Range and Velocity of target]
- [FMCW Waveform Generation]
- [Signal generation and Moving Target simulation]
- [RANGE MEASUREMENT]
- [RANGE DOPPLER RESPONSE]
- [CFAR implementation]

</div>

```{.codeinput}
clear all;
clc;
```

## Radar Specifications 

```{.codeinput}
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Frequency of operation = 77GHz
% Max Range = 200m
% Range Resolution = 1 m
% Max Velocity = 100 m/s
%%%%%%%%%%%%%%%%%%%%%%%%%%%
Range_res = 1;
Range_max = 200;
Velocity_max = 100;
```

## User Defined Range and Velocity of target

**%TODO** : define the target's initial position and velocity. Note :
Velocity remains contant

```{.codeinput}
% *%TODO* :
% Range cannot exceed the max value of 200m 
% and velocity can be any value in the range of -70 to + 70 m/s.
 
Range=110; % Initial distance of the target
Velocity=-20; % Initial speed of the target
```

## FMCW Waveform Generation

```{.codeinput}

% *%TODO* :
% Design the FMCW waveform by giving the specs of each of its parameters.
% Calculate the Bandwidth (B), Chirp Time (Tchirp) and Slope (slope) of the FMCW
% chirp using the requirements above.

% Bandwidth for each chirp for given resolution
Bandwidth=c/(2*Range_res);

% Sweep time for each chirp is defined as rule by 5.5 times round trip time
% for maximum range
Tchirp = 5.5 * 2 * Range_max / c;

% The slope of the chirp
Slope = Bandwidth / Tchirp; 

% Operating carrier frequency of Radar 
fc = 77e9;             %carrier freq
                                                          
% The number of chirps in one sequence. Its ideal to have 2^ value for the ease of running the FFT
% for Doppler Estimation. 
Nd = 128;                   % #of doppler cells OR #of sent periods % number of chirps

% The number of samples on each chirp. 
Nr = 1024;                  %for length of time OR # of range cells

% Timestamp for running the displacement scenario for every sample on each
% chirp
t = linspace(0, Nd*Tchirp, Nr*Nd); %total time for samples


%Creating the vectors for Tx, Rx and Mix based on the total samples input.
Tx = zeros(1, length(t)); %transmitted signal
Rx = zeros(1, length(t)); %received signal
Mix = zeros(1, length(t)); %beat signal

%Similar vectors for range_covered and time delay.
r_t = zeros(1, length(t));
td = zeros(1, length(t));
```

## Signal generation and Moving Target simulation


```{.codeinput}
%% Running the radar scenario over the time.
%% Signal generation and Moving Target simulation
% Running the radar scenario over the time. 

for i=1:length(t)         
    
    % *%TODO* :
    % For each time stamp update the Range of the Target for constant velocity. 
    r_t(i) = Range + Velocity*t(i); 
    td(i) = r_t(i)*2 / c;
    
    % *%TODO* :
    % For each time sample we need update the transmitted and
    % received signal. 
    Tx(i) = cos( 2*pi * (fc * t(i) + Slope * 0.5 * t(i)^2) );
    Rx(i)  = cos( 2*pi * (fc * (t(i) - td(i)) + Slope * 0.5 * (t(i) - td(i))^2) );
    
    % *%TODO* :
    % Now by mixing the Transmit and Receive generate the beat signal
    % This is done by element wise matrix multiplication of Transmit and
    % Receiver Signal
    Mix(i) = Tx(i).*Rx(i); % Beat signal 
    
end
```

## RANGE MEASUREMENT

**%TODO** :

```{.codeinput}
%% RANGE MEASUREMENT

% *%TODO* :
% reshape the vector into Nr*Nd array. Nr and Nd here would also define the size of
% Range and Doppler FFT respectively.
Mix = reshape(Mix, [Nr, Nd]);

 % *%TODO* :
% run the FFT on the beat signal along the range bins dimension (Nr) and
% normalize.
sig_fft = fft(Mix, Nr); 

% *%TODO* :
% Take the absolute value of FFT output
sig_fft = abs(sig_fft);
sig_fft = sig_fft ./ max(sig_fft); % Normalizing the signal

% *%TODO* :
% Output of FFT is double sided signal, but we are interested in only one side of the spectrum.
% Hence we throw out half of the samples.
sig_fft = sig_fft( 1 : (Nr/2) );

% plotting the range
% *%TODO* :
% plot FFT output 
figure ('Name','Range from First FFT')
plot(sig_fft);
title('Range from First 1D FFT');
ylabel('Normalized Amplitude');
xlabel('Range');
axis ([0 Range_max 0 1]);
```

## RANGE DOPPLER RESPONSE 

---

The 2D FFT implementation is already provided here. This will run a
2DFFT on the mixed signal (beat signal) output and generate a range
doppler map.You will implement CFAR on the generated RDM

```{.codeinput}
%% RANGE DOPPLER RESPONSE
% The 2D FFT implementation is already provided here. This will run a 2DFFT
% on the mixed signal (beat signal) output and generate a range doppler
% map (RDM). You will implement CFAR on the generated RDM.

% Range Doppler Map Generation

% The output of the 2D FFT is an image that has reponse in the range and
% doppler FFT bins. So, it is important to convert the axis from bin sizes
% to range and doppler based on their Max values.
Mix = reshape(Mix,[Nr,Nd]);

% 2D FFT using the FFT size for both dimensions.
sig_fft2 = fft2(Mix,Nr,Nd);

% Taking just one side of signal from Range dimension.
sig_fft2 = sig_fft2(1:Nr/2,1:Nd);
sig_fft2 = fftshift (sig_fft2);
RDM = abs(sig_fft2);
RDM = 10*log10(RDM);

%use the surf function to plot the output of 2DFFT and to show axis in both
%dimensions
doppler_axis = linspace(-100,100,Nd);
range_axis = linspace(-200,200,Nr/2)*((Nr/2)/400);
figure,surf(doppler_axis,range_axis,RDM);
title('2D FFT Output - Range Doppler Map (RDM)');
ylabel('Range');
xlabel('Velocity');
zlabel('Signal Strength');
```

## CFAR implementation

---

```{.codeinput}
%% CFAR implementation

% Slide Window through the complete Range Doppler Map

% *%TODO* :
% Select the number of Training Cells in both the dimensions.
Tr = 10;
Td = 8;

% *%TODO* :
% Select the number of Guard Cells in both dimensions around the Cell under 
% test (CUT) for accurate estimation
Gr = 4;
Gd = 4;

% *%TODO* :
% offset the threshold by SNR value in dB
offset = 1.2;

% *%TODO* :
%Create a vector to store noise_level for each iteration on training cells

% *%TODO* :
% design a loop such that it slides the CUT across range doppler map by
% giving margins at the edges for Training and Guard Cells.
% For every iteration sum the signal level within all the training
% cells. To sum convert the value from logarithmic to linear using db2pow
% function. Average the summed values for all of the training
% cells used. After averaging convert it back to logarithimic using pow2db.
% Further add the offset to it to determine the threshold. Next, compare the
% signal under CUT with this threshold. If the CUT level > threshold assign
% it a value of 1, else equate it to 0.

% Use RDM[x,y] as the matrix from the output of 2D FFT for implementing
% CFAR
RDM = RDM / max(RDM(:));

for rows = (Tr + Gr + 1) : (Nr / 2) -  (Tr + Gr)
    for cols = (Td + Gd + 1) : Nd - (Td + Gd)
        noise_level = zeros(1,1);
        for cell_in_rows = rows - (Tr + Gr) : rows + (Tr + Gr)
            for cell_in_cols = cols - (Td + Gd) : cols + (Td + Gd)
                if( abs(rows - cell_in_rows) > Gr || abs(cols - cell_in_cols) > Gd )
                    noise_level = noise_level + db2pow(RDM(cell_in_rows, cell_in_cols));
                end
            end 
        end
        Threshold = noise_level / (2*(Td+Gd+1)*2*(Tr+Gr+1)-(Gr*Gd)-1);
        Threshold = pow2db(Threshold);
        Threshold = Threshold + offset;
        
        CUT = RDM(rows, cols);
        if CUT > Threshold
            RDM(rows, cols) = 1;
        else
            RDM(rows, cols) = 0;
        end
    end
end
        
% *%TODO* :
% The process above will generate a thresholded block, which is smaller 
% than the Range Doppler Map as the CUT cannot be located at the edges of
% matrix. Hence, few cells will not be thresholded. To keep the map size same
% set those values to 0. 
RDM(RDM~=0 & RDM~=1) = 0;

% *%TODO* :
% display the CFAR output using the Surf function like we did for Range
% Doppler Response output.
figure, surf(doppler_axis, range_axis, RDM);
colorbar;
title( 'CA-CFAR Filtered RDM surface plot');
xlabel('Velocity');
ylabel('Range');
zlabel('Amplitude');
```
<img src="https://github.com/MuhammadKA/Sensor-Fusion-ND/blob/master/3-%20Radar:%20Target%20Generation%20and%20Detection/1D_FFT_Output.png" width="400" height="400" />
<img src="https://github.com/MuhammadKA/Sensor-Fusion-ND/blob/master/3-%20Radar:%20Target%20Generation%20and%20Detection/2D_FFT_Output.png" width="400" height="400" />
<img src="https://github.com/MuhammadKA/Sensor-Fusion-ND/blob/master/3-%20Radar:%20Target%20Generation%20and%20Detection/CA_CFAR_Filtered_RDM_surface_plot.png" width="400" height="400" />
</div>

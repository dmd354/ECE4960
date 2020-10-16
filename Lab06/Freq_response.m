data = table2array(readtable('tap_data.csv'));
Ts = mean(data(:,2))*(10^-6); % average sampling time in seconds
Fs = 1/Ts;  % sampling frequency

X = data(:,1);  % accelerometer data

% plot(X)         % check raw data
% X = X(1:end);   %trim data

L = length(X);  % Length of data
Y = fft(X); % compute Fourier Transform

% Compute the two-sided spectrum P2. Then compute the single-sided spectrum P1 based on P2 and the even-valued signal length L.
P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs*(0:(L/2))/L; % frequency vector
plot(f,P1) 
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')
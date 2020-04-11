% 2-D Transform
% The 2-D Fourier transform is useful for processing 2-D signals and other 2-D data such as images.
% Create and plot 2-D data with repeated blocks.
M = 5; % size of range FFT samples
N = 10; % size of dopplar FFT samples

P = peaks(20);
X = repmat(P, [M N]);
imagesc(X);

% Compute the 2-D Fourier transform of the data.  
% Shift the zero-frequency component to the center of the output, and 
% plot the resulting 100-by-200 matrix, which is the same size as X.
Y = fft2(X); % 2D FFT
Y = fftshift(Y); % shift zero frequency terms to the center of the array
Y = abs(Y); % don't care about negative (non-real) frequencies
imagesc(Y);

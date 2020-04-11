% Implement 1D CFAR using lagging cells on the given noise and target scenario.
close all;
clear;

Ns = 1000; % number of data points (range bins)
s = abs(randn(Ns, 1)); % generate random noise
s([100, 200, 350, 700]) = [8 9 5 11]; % add four targets
plot(s);

T = 12; % training cells (lagging)
G = 4; % guard cells
snr_offset = 5.5; % offset above noise threshold for desired signal to noise ratio
threshold_cfar = [];
signal_cfar = [];

for i = 1:(Ns-(G+T))
    noise_level = sum(s(i:i+T-1)); % sum of noise in training cells

    threshold = (noise_level / T) * snr_offset; % linear data so multiply with the offset (add for logarithic data)
    threshold_cfar = [threshold_cfar, {threshold}];

    signal = s(i+T+G);

    if signal < threshold
        signal = 0;
    end

    signal_cfar = [signal_cfar, {signal}];
end

% plot the filtered signal
plot (cell2mat(signal_cfar), 'g--');

% plot original sig, threshold and filtered signal within the same figure.
figure, plot(s);
hold on, plot(cell2mat(circshift(threshold_cfar, G)),'r--', 'LineWidth', 2);
hold on, plot(cell2mat(circshift(signal_cfar, (T+G))),'g--', 'LineWidth', 4);
legend('Signal', 'CFAR Threshold', 'detection');

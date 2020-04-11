% Lesson 3 - Section 2
c = 3 * 10 ^ 8; % speed of light [m/s]
range_resolution = 1; % [m]
Bsweep = c / (2 * range_resolution); % [Hz]

max_range = 300; % [m]
sweep_factor = 5.5; % sweep time should be 5-6 times the round trip time at max range
Ts = sweep_factor * 2 * max_range / c; % sweep/chirp time

fb = [0 1.1e6 13e6 24e6]; % beat frequencies (ramping frequeny - received frequency) for four targets [Hz]
target_ranges = (c * Ts *fb) / (2 * Bsweep); % [m]
display(target_ranges); % expect ranges less than the max range

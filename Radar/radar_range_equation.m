% Lesson 2 - Section 7
c = 3 * 10 ^ 8; % speed of light [m/s]
fc = 77.0e9; % operating frequency [Hz]

lambda = c / fc; % wavelength [m]
display(lambda); % expect wavelength of millimeters

Pt = 3e-3; % tranmitted power [W]
G = 10000; % antenna gain (linear) [no units]
RCS = 100; % RCS (car) [m ^ 2]
Pe = 1e-10; % minimum detectable power [W]

max_range = nthroot((Pt * (G ^ 2) * (lambda ^ 2) * RCS) / (Pe * ((4 * pi) ^ 3)), 4); % max range of radar [m]
display(max_range); % expect max range to be in the 100's of meters

% Lesson 3 - Section 3
c = 3 * 10 ^ 8; % speed of light [m/s]
f = 77.0e9; % operating frequency [Hz]
lambda = c / f; % wavelength [m]

fd = [3e3 4.5e3 11e3 -3e3]; % dopplar shifts for four targets [Hz]
vr = (fd * lambda) / 2; % target velocities [m/s]
display(vr); % positive = approaching | negative = receding

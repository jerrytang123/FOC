% Init
clear; clc; close all;

% Controller parameters
PID_LOOP_PERIOD = 250;  % us
ALPHA_VELOCITY_TIME_CONST_US = 10000;  % us, low-pass filter time constant for velocity measurement

% Simulation parameters
SIMULATION_TIME = 2e5;
IMPULSE_START_TIME = SIMULATION_TIME/2;
IMPULSE_STOP_TIME = IMPULSE_START_TIME + PID_LOOP_PERIOD;
IMPULSE_MAGNITUDE = 1;

% Init simulation
time = 0:PID_LOOP_PERIOD:SIMULATION_TIME;
n = length(time);
velocity_raw = zeros(1, n);
velocity_filtered = zeros(1, n);

% Run simulation
for i = 1:n
    if time(i) >= IMPULSE_START_TIME && time(i) < IMPULSE_STOP_TIME
        velocity_raw(i) = IMPULSE_MAGNITUDE;
    end
    if i > 1
        delta_time_us = PID_LOOP_PERIOD;
        alpha_velocity_sense = delta_time_us / (delta_time_us + ALPHA_VELOCITY_TIME_CONST_US);
        velocity_filtered(i) = alpha_velocity_sense * velocity_raw(i) + (1 - alpha_velocity_sense) * velocity_filtered(i - 1);
    end
end

% Plot results
figure(1);
plot(time./1000, velocity_raw, 'b');
hold on;
plot(time./1000, velocity_filtered, 'r');
grid on;
xlabel('Time (ms)');
ylabel('Velocity (m/s)');
legend('Raw', 'Filtered');
title('Velocity measurement');

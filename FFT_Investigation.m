%% FFT Comparison: Before vs After Filter
% This script compares the frequency spectra of Sharpness and Servo Position
% before and after a filtering process.

clear; clc; close all;

%% --- Input CSV file names ---
file_before = 'focus_log2.csv';   % <-- change to your actual filename
file_after  = 'focus_log.csv';    % <-- change to your actual filename

%% --- Read both CSV files ---
data_before = readtable(file_before);
data_after  = readtable(file_after);

% --- Detect columns automatically (case-insensitive) ---
time_b = data_before{:, find(contains(lower(data_before.Properties.VariableNames), 'time'))};
sharp_b = data_before{:, find(contains(lower(data_before.Properties.VariableNames), 'sharp'))};
servo_b = data_before{:, find(contains(lower(data_before.Properties.VariableNames), 'servo'))};

time_a = data_after{:, find(contains(lower(data_after.Properties.VariableNames), 'time'))};
sharp_a = data_after{:, find(contains(lower(data_after.Properties.VariableNames), 'sharp'))};
servo_a = data_after{:, find(contains(lower(data_after.Properties.VariableNames), 'servo'))};

%% --- Ensure equal length and sampling rate ---
N = min(length(time_b), length(time_a));
time_b = time_b(1:N);
time_a = time_a(1:N);
sharp_b = sharp_b(1:N);
sharp_a = sharp_a(1:N);
servo_b = servo_b(1:N);
servo_a = servo_a(1:N);

Ts = mean(diff(time_b));     % sampling interval
Fs = 1/Ts;                   % sampling frequency
f = Fs*(0:(N/2)-1)/N;        % frequency vector (single-sided)

%% --- FFT function handle ---
compute_fft = @(signal, N) 2*abs(fft(signal)/N);
fft_b_sharp = compute_fft(sharp_b, N);
fft_a_sharp = compute_fft(sharp_a, N);
fft_b_servo = compute_fft(servo_b, N);
fft_a_servo = compute_fft(servo_a, N);

% Keep only single-sided spectrum
fft_b_sharp = fft_b_sharp(1:N/2);
fft_a_sharp = fft_a_sharp(1:N/2);
fft_b_servo = fft_b_servo(1:N/2);
fft_a_servo = fft_a_servo(1:N/2);

%% --- Plot Sharpness FFTs ---
figure;
plot(f, fft_b_sharp, 'r', 'LineWidth', 1.2); hold on;
plot(f, fft_a_sharp, 'b', 'LineWidth', 1.2);
grid on;
xlabel('Frequency (Hz)');
ylabel('Amplitude');
title('FFT of Sharpness: Before vs After Filter');
legend('Before Filter', 'After Filter');

%% --- Plot Servo Position FFTs ---
figure;
plot(f, fft_b_servo, 'r', 'LineWidth', 1.2); hold on;
plot(f, fft_a_servo, 'b', 'LineWidth', 1.2);
grid on;
xlabel('Frequency (Hz)');
ylabel('Amplitude');
title('FFT of Servo Position: Before vs After Filter');
legend('Before Filter', 'After Filter');

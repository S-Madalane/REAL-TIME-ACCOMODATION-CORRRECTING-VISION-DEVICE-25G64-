%% compare_autofocus_performance.m
% Compare sharpness performance with and without autofocus system

clc; clear; close all;

%% ---------- Data ----------

% Distances (in meters)
distance = [0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0 1.1 1.2 1.3 1.4 1.5 1.6 1.7 1.8];

% WITH autofocus system
sharp_ESD = [0.425 0.634 0.652 0.660 0.658 0.637 0.609 0.605 0.612 0.627 0.610 0.616 0.621 0.624 0.621 0.622 0.622 0.622 0.622];
sharp_DLAB = [0.580 0.664 0.682 0.694 0.698 0.689 0.675 0.656 0.661 0.658 0.645 0.635 0.629 0.620 0.618 0.615 0.608 0.602 0.600];

% WITHOUT autofocus system
sharp_noAF = [0.09 0.10 0.20 0.20 0.22 0.30 0.31 0.34 0.34 0.36 0.37 0.37 0.38 0.39 0.34 0.35 0.36 0.37 0.36];

%% ---------- Plot ----------
figure('Color','w');
hold on; grid on; box on;

plot(distance, sharp_ESD, '-o', 'LineWidth', 1.6, 'Color', [0.1 0.4 0.8], 'DisplayName', 'ESD LAB (Autofocus at 280 Lux)');
plot(distance, sharp_DLAB, '-s', 'LineWidth', 1.6, 'Color', [0.8 0.2 0.2], 'DisplayName', 'D-LAB (Autofocus at 320 Lux)');
plot(distance, sharp_noAF, '--d', 'LineWidth', 1.4, 'Color', [0.3 0.3 0.3], 'DisplayName', 'Without Autofocus');

xlabel('Object Distance (m)', 'FontSize', 12);
ylabel('Sharpness Score', 'FontSize', 12);
title('Comparison of Sharpness With and Without Autofocus System', 'FontWeight', 'bold');
legend('Location','best');
ylim([0 0.8]);

%% ---------- Percentage Improvement ----------
% Interpolate "without AF" data to match autofocus distances
sharp_noAF_interp = interp1(distance, sharp_noAF, distance, 'linear', 'extrap');

% Compute average sharpness values
mean_noAF = mean(sharp_noAF_interp);
mean_withAF = mean([sharp_ESD sharp_DLAB], 'all');

% Compute percentage increase
percent_increase = ((mean_withAF - mean_noAF) / mean_noAF) * 100;

fprintf('\n=== SHARPNESS PERFORMANCE SUMMARY ===\n');
fprintf('Mean Sharpness (Without AF): %.3f\n', mean_noAF);
fprintf('Mean Sharpness (With AF): %.3f\n', mean_withAF);
fprintf('Overall %% Increase in Sharpness: %.2f%%\n', percent_increase);

% Annotate on plot
%text(0.2, 0.75, sprintf('Average Sharpness Increase = %.1f%%', percent_increase), ...
   % 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.1 0.5 0.1]);

disp('✓ Plot and analysis complete.');

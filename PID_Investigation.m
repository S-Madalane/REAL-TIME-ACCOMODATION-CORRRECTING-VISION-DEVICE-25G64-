for i = 1:10
    filename = sprintf('servo_time_log%d.csv', i);
    data = readtable(filename);

    % --- Detect column names automatically ---
    time_col_idx = find(contains(lower(data.Properties.VariableNames), 'time'));
    servo_col_idx = find(contains(lower(data.Properties.VariableNames), 'servo'));

    time = data{:, time_col_idx};
    servo = data{:, servo_col_idx};

    % --- Filter: only keep data between 2s and 7s ---
    mask = (time >= 2) & (time <= 7);
    time_filtered = time(mask);
    servo_filtered = servo(mask);

    % --- New figure for each run ---
    figure;
    plot(time_filtered, servo_filtered);
    title(sprintf('Servo Position vs Time (Run %d, 2s–7s)', i));
    xlabel('Time (s)');
    ylabel('Servo Position (°)');
    grid on;
end

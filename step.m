%% --- Robust CSV Load ---
filename = 'servo_time_log10.csv';  % replace with your CSV file

% Try reading with headers
try
    data_csv = readtable(filename);
    varnames = data_csv.Properties.VariableNames;
    % Attempt to find time and servo columns automatically
    time_col  = find(contains(lower(varnames), 'time'),1);
    servo_col = find(contains(lower(varnames), 'servo'),1);
    
    if isempty(time_col) || isempty(servo_col)
        error('Column names not found.');
    end
    
    time_csv  = data_csv{:,time_col};
    servo_csv = data_csv{:,servo_col};
    
catch
    % Fallback: treat as CSV without headers
    data_csv = readtable(filename,'ReadVariableNames',false);
    time_csv  = data_csv.Var1;  % first column
    servo_csv = data_csv.Var3;  % third column (adjust if different)
end

%% --- System Parameters ---
zeta = 0.5; wn = 15; K_servo = 1;
num_servo = K_servo*wn^2;
den_servo = [1 2*zeta*wn wn^2];
G_servo = tf(num_servo, den_servo);

% PID
Kp = 2000; Ki = 0.25; Kd = 5;
C_pid = pid(Kp, Ki, Kd);
T_closed = feedback(C_pid*G_servo, 1);

%% --- Sharpness Function ---
theta_opt = 105; sigma = 15;
sharpness_fn = @(theta) exp(-(theta - theta_opt).^2/(2*sigma^2));

%% --- Smart Coarse Sweep ---
servo_min = 5; servo_max = 175; coarse_step = 10;
init_pos = 90;
pos = init_pos;
coarse_positions = pos;
sharp_prev = sharpness_fn(pos);
max_steps = 20;

% Sweep upwards
for i = 1:max_steps
    next_pos = min(pos + coarse_step, servo_max);
    sharp_next = sharpness_fn(next_pos);
    if sharp_next < sharp_prev
        break;
    end
    pos = next_pos;
    coarse_positions(end+1) = pos;
    sharp_prev = sharp_next;
end

% Sweep downwards
pos = init_pos; sharp_prev = sharpness_fn(pos);
for i = 1:max_steps
    next_pos = max(pos - coarse_step, servo_min);
    sharp_next = sharpness_fn(next_pos);
    if sharp_next < sharp_prev
        break;
    end
    pos = next_pos;
    coarse_positions = [next_pos coarse_positions]; % prepend
    sharp_prev = sharp_next;
end

coarse_sharpness = arrayfun(sharpness_fn, coarse_positions);
[~, idx_best_coarse] = max(coarse_sharpness);
best_coarse_pos = coarse_positions(idx_best_coarse);

%% --- Fine Sweep ---
fine_range = 12; fine_step = 2;
fine_positions = max(servo_min,best_coarse_pos-fine_range):fine_step:...
                 min(servo_max,best_coarse_pos+fine_range);
fine_sharpness = arrayfun(sharpness_fn, fine_positions);
[~, idx_best_fine] = max(fine_sharpness);
best_fine_pos = fine_positions(idx_best_fine);

fprintf('Best coarse: %.1f°, Best fine: %.1f°\n', best_coarse_pos, best_fine_pos);

%% --- Build Timeline (0-7 s) ---
t_total = 0:0.001:7;
u_total = zeros(size(t_total));

% Sweep shift time
t_shift = 4.3; sweep_time_step = 0.02;

% Coarse sweep times
t_coarse = t_shift:sweep_time_step:(t_shift + sweep_time_step*(length(coarse_positions)-1));
u_coarse = coarse_positions;

% Fine sweep times
t_fine = t_coarse(end)+sweep_time_step:sweep_time_step:...
         t_coarse(end)+sweep_time_step*length(fine_positions);
u_fine = fine_positions;

% Concatenate
t_sweep = [t_coarse t_fine];
u_sweep = [u_coarse u_fine];

% Fill input vector
for k = 1:length(t_sweep)
    idx = find(t_total >= t_sweep(k),1,'first');
    u_total(idx:end) = u_sweep(k);
end

% Hold final target after sweep
idx_final = find(t_total >= t_sweep(end),1,'first');
u_total(idx_final:end) = best_fine_pos;

%% --- Simulate Response ---
y_response = lsim(T_closed, u_total, t_total);

%% --- Plot Servo Step Response + CSV Data (No Input) ---
figure;
plot(t_total, y_response, 'LineWidth', 2, 'DisplayName', 'Simulated Servo Output'); hold on;

% Plot CSV data
plot(time_csv, servo_csv, 'LineWidth' ,2, 'DisplayName', 'Measured Servo');

xlabel('Time (s)'); ylabel('Servo Angle (°)');
title('Servo Response: Simulation vs Measured');
grid on; xlim([0 7]); ylim([0 180]);
legend('Location','best');

%% --- Plot Sharpness vs Servo Angle ---
figure;
plot(coarse_positions, coarse_sharpness,'ro-','LineWidth',1.5); hold on;
plot(fine_positions, fine_sharpness,'bo-','LineWidth',1.5);
xline(best_fine_pos,'k--','LineWidth',2,'DisplayName','Optimal Angle');
xlabel('Servo Angle (°)'); ylabel('Sharpness');
title('Sharpness vs Servo Angle');
legend('Coarse Sweep','Fine Sweep','Optimal Angle');
grid on;

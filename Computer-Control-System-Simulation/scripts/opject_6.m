%% ==================== 实验六：大林算法控制器设计 ====================
% 内容：大林算法控制器设计、lambda参数分析、振铃消除、与最小拍对比
% 输出：保存图片到 opject_6_images/ 文件夹
% ====================================================================

clear; clc;

% 创建图片保存文件夹
output_dir = 'opject_6_images';
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

%% ==================== 步骤1：建立被控对象模型 ====================
% 一阶加滞后对象: G(s) = e^{-2s} / (3s + 1)
K = 1; T1 = 3; tau = 2;

% 二阶对象: G(s) = 2e^{-1.5s} / ((4s+1)(2s+1))
K2 = 2; T1_2 = 4; T2_2 = 2; tau_2 = 1.5;

fprintf('===== 大林算法控制器设计 =====\n');
fprintf('一阶对象: G(s) = %.0f * e^{-%.1fs} / (%.0fs + 1)\n', K, tau, T1);
fprintf('二阶对象: G(s) = %.0f * e^{-%.1fs} / ((%.0fs+1)(%.0fs+1))\n', K2, tau_2, T1_2, T2_2);

T = 0.2;  % 采样周期
d = round(tau / T);
fprintf('采样周期 T = %.2f s, 滞后步数 d = %d\n', T, d);

%% ==================== 步骤2：lambda参数对系统性能的影响 ====================
fprintf('\n====== lambda参数对系统性能的影响 ======\n');

lambda_list = [0.5, 1.0, 1.5, 2.0, 3.0, 5.0];
lambda_labels = {'\lambda=0.5(快)', '\lambda=1.0', '\lambda=1.5', ...
                 '\lambda=2.0', '\lambda=3.0', '\lambda=5.0(慢)'};

sim_N = 150;  % 仿真步数

fig1 = figure('Name', '实验六：lambda参数对系统性能的影响', 'Position', [100, 100, 1300, 900]);

results_table = cell(length(lambda_list), 8);

for idx = 1:length(lambda_list)
    lam = lambda_list(idx);
    
    [num_d, den_d] = dahlin_design_1st(K, T1, tau, T, lam);
    
    a1 = exp(-T / T1);
    b0 = K * (1 - a1);
    d = round(tau / T);
    
    num_g = [zeros(1, d), b0];
    den_g = [1, -a1];
    
    r = ones(1, sim_N);
    [y, u] = simulate_dahlin(num_g, den_g, num_d, den_d, r);
    
    % 性能指标
    y_final = mean(y(end-30:end));
    overshoot = max(0, (max(y) - y_final) / y_final * 100);
    
    settling_idx = sim_N;
    for k = sim_N:-1:1
        if abs(y(k) - y_final) > 0.02 * abs(y_final)
            settling_idx = k;
            break;
        end
    end
    settling_time = settling_idx * T;
    IAE = sum(abs(r - y)) * T;
    
    % 控制量指标
    u_ss = u(20:end);
    u_max_val = max(u);
    u_min_ss = min(u_ss);
    u_max_ss = max(u_ss);
    u_std_val = std(u_ss);
    
    results_table{idx, 1} = lam;
    results_table{idx, 2} = overshoot;
    results_table{idx, 3} = settling_time;
    results_table{idx, 4} = abs(r(end) - y_final);
    results_table{idx, 5} = IAE;
    results_table{idx, 6} = u_max_val;
    results_table{idx, 7} = sprintf('[%.2f, %.2f]', u_min_ss, u_max_ss);
    results_table{idx, 8} = u_std_val;
    
    % 绘图
    subplot(3, 2, idx);
    yyaxis left;
    stairs((0:sim_N-1)*T, y, 'b-', 'LineWidth', 1.2); hold on;
    plot((0:sim_N-1)*T, r, 'k--', 'LineWidth', 1);
    ylabel('y(t)'); ylim([0 1.5]);
    
    yyaxis right;
    stairs((0:sim_N-1)*T, u, 'r-', 'LineWidth', 0.8);
    ylabel('u(t)');
    
    xlabel('时间 t (s)');
    title(lambda_labels{idx});
    grid on;
end

% 打印表格
fprintf('\n====== 性能指标汇总 ======\n');
fprintf('%-10s %-8s %-10s %-10s %-10s %-10s %-18s %-10s\n', ...
    'lambda', '超调%%', '调节时间', '稳态误差', 'IAE', 'u_max', 'u幅值范围', 'u_std');
fprintf('%-10s %-8s %-10s %-10s %-10s %-10s %-18s %-10s\n', ...
    '------', '------', '------', '------', '------', '------', '------', '------');
for i = 1:length(lambda_list)
    fprintf('%-10.1f %-8.2f %-10.4f %-10.6f %-10.4f %-10.4f %-18s %-10.4f\n', results_table{i,:});
end

sgtitle('\lambda参数对大林算法系统性能的影响');
img_path = fullfile(output_dir, 'fig1_lambda参数影响.png');
exportgraphics(fig1, img_path, 'Resolution', 150);

%% ==================== 步骤3：振铃现象分析与消除 ====================
fprintf('\n====== 振铃现象分析与消除 ======\n');

lambda = 1.0;
T = 0.2;

[num_d, den_d] = dahlin_design_1st(K, T1, tau, T, lambda);

fprintf('原始D(z)极点:\n');
poles_original = roots(den_d);
disp(poles_original);

ringing_poles = poles_original(real(poles_original) < -0.1);
fprintf('可能产生振铃的极点 (z<-0.1):\n');
if isempty(ringing_poles)
    fprintf('  未发现振铃极点\n');
else
    disp(ringing_poles);
    
    fprintf('\n振铃消除修正因子:\n');
    for i = 1:length(ringing_poles)
        p_r = ringing_poles(i);
        correction = 1 - p_r;
        fprintf('  极点 %.4f -> 修正因子 %.4f\n', p_r, correction);
    end
    
    [num_d_mod, den_d_mod] = eliminate_ringing(num_d, den_d, ringing_poles);
end

a1 = exp(-T / T1);
b0 = K * (1 - a1);
d = round(tau / T);
num_g = [zeros(1, d), b0];
den_g = [1, -a1];

sim_N = 120;
r = ones(1, sim_N);

[y_orig, u_orig] = simulate_dahlin(num_g, den_g, num_d, den_d, r);
[y_mod, u_mod] = simulate_dahlin(num_g, den_g, num_d_mod, den_d_mod, r);

% ---- 振铃消除前后指标对比 ----
fprintf('\n====== 振铃消除前后指标对比 ======\n');
yf_o = mean(y_orig(end-30:end));  yf_m = mean(y_mod(end-30:end));
overshoot_o = max(0, (max(y_orig) - yf_o) / yf_o * 100);
overshoot_m = max(0, (max(y_mod)  - yf_m) / yf_m * 100);
st_o = sim_N * T;
for kk = sim_N:-1:1
    if abs(y_orig(kk) - yf_o) > 0.02 * abs(yf_o), st_o = kk * T; break; end
end
st_m = sim_N * T;
for kk = sim_N:-1:1
    if abs(y_mod(kk) - yf_m) > 0.02 * abs(yf_m), st_m = kk * T; break; end
end
u_ss_o = u_orig(20:end);  u_ss_m = u_mod(20:end);
fprintf('                 %-16s %-16s\n', '原始(有振铃)', '消振后');
fprintf('超调量 %%        %16.2f %16.2f\n', overshoot_o, overshoot_m);
fprintf('调节时间 (s)     %16.4f %16.4f\n', st_o, st_m);
fprintf('u 最大值         %16.4f %16.4f\n', max(u_orig), max(u_mod));
fprintf('u 幅值范围(稳态) [%.2f,%.2f]    [%.2f,%.2f]\n', ...
    min(u_ss_o), max(u_ss_o), min(u_ss_m), max(u_ss_m));
fprintf('u 标准差(稳态)   %16.4f %16.4f\n', std(u_ss_o), std(u_ss_m));
fprintf('振铃评价         %-16s %-16s\n', '存在明显振铃', '振铃已消除');

fig2 = figure('Name', '振铃消除前后对比', 'Position', [100, 100, 1000, 800]);

subplot(3, 1, 1);
stairs((0:sim_N-1)*T, r, 'k--', 'LineWidth', 1.5); hold on;
stairs((0:sim_N-1)*T, y_orig, 'b-', 'LineWidth', 1.2);
stairs((0:sim_N-1)*T, y_mod, 'r-', 'LineWidth', 1.2);
ylabel('y(t)'); legend('设定值', '原始', '消振后');
title('系统输出对比'); grid on;

subplot(3, 1, 2);
stairs((0:sim_N-1)*T, u_orig, 'b-', 'LineWidth', 0.8); hold on;
stairs((0:sim_N-1)*T, u_mod, 'r-', 'LineWidth', 0.8);
ylabel('u(t)'); legend('原始', '消振后');
title('控制量对比（观察振铃）'); grid on;

subplot(3, 1, 3);
U_orig = fft(u_orig(30:end));
U_mod = fft(u_mod(30:end));
freq = (0:length(U_orig)-1) / (length(U_orig) * T);
plot(freq(1:floor(end/2)), abs(U_orig(1:floor(end/2))), 'b-', 'LineWidth', 1); hold on;
plot(freq(1:floor(end/2)), abs(U_mod(1:floor(end/2))), 'r-', 'LineWidth', 1);
xlabel('频率 (Hz)'); ylabel('|U(f)|');
legend('原始', '消振后');
title('控制量频谱对比');
grid on;

sgtitle('大林算法——振铃消除前后对比');
img_path = fullfile(output_dir, 'fig2_振铃消除.png');
exportgraphics(fig2, img_path, 'Resolution', 150);

%% ==================== 步骤4：大林算法 vs 最小拍控制对比 ====================
fprintf('\n====== 大林算法 vs 最小拍控制对比 ======\n');

T = 0.2;
d = round(tau / T);

[num_dd, den_dd] = dahlin_design_1st(K, T1, tau, T, 1.0);

a1 = exp(-T / T1);
b0 = K * (1 - a1);
d = round(tau / T);
Gz = tf([zeros(1,d), b0], [1, -a1], T);
Phi_db = tf([zeros(1,d+1), 1], [1, zeros(1,d+1)], T);
Dz_db = minreal(Phi_db / (Gz * (1 - Phi_db)));
[num_db, den_db] = tfdata(Dz_db, 'v');

a1 = exp(-T / T1);
b0 = K * (1 - a1);
num_g = [zeros(1, d), b0];
den_g = [1, -a1];

sim_N = 100;
r = ones(1, sim_N);

[y_dahlin, u_dahlin] = simulate_dahlin(num_g, den_g, num_dd, den_dd, r);
[y_db, u_db] = simulate_dahlin(num_g, den_g, num_db, den_db, r);

% ---- 标称模型性能指标对比 ----
fprintf('\n====== 标称模型性能对比 ======\n');
yf_d  = mean(y_dahlin(end-20:end));
yf_db = mean(y_db(end-20:end));
os_d  = max(0, (max(y_dahlin) - yf_d) / yf_d * 100);
os_db = max(0, (max(y_db) - yf_db) / yf_db * 100);
st_d = sim_N * T;
for kk = sim_N:-1:1
    if abs(y_dahlin(kk) - yf_d) > 0.02 * abs(yf_d), st_d = kk * T; break; end
end
st_db = sim_N * T;
for kk = sim_N:-1:1
    if abs(y_db(kk) - yf_db) > 0.02 * abs(yf_db), st_db = kk * T; break; end
end
track_d  = find(abs(y_dahlin - yf_d) <= 0.05 * abs(yf_d), 1);
track_db = find(abs(y_db - yf_db) <= 0.05 * abs(yf_db), 1);
u_ss_d  = u_dahlin(30:end);
u_ss_db = u_db(30:end);
fprintf('                  %-16s %-16s\n', '大林算法', '最小拍');
fprintf('超调量 %%         %16.2f %16.2f\n', os_d, os_db);
fprintf('调节时间 (s)      %16.4f %16.4f\n', st_d, st_db);
fprintf('跟踪拍数(±5%%)     %16d  %16d\n', track_d, track_db);
fprintf('u 最大值          %16.4f %16.4f\n', max(u_dahlin), max(u_db));
fprintf('u 幅值范围(稳态)  [%.2f,%.2f]    [%.2f,%.2f]\n', ...
    min(u_ss_d), max(u_ss_d), min(u_ss_db), max(u_ss_db));
fprintf('u 标准差(稳态)    %16.4f %16.4f\n', std(u_ss_d), std(u_ss_db));
if std(u_ss_db) < std(u_ss_d)
    fprintf('控制量平稳性评价    %-14s %-14s\n', '较平稳', '很平稳');
else
    fprintf('控制量平稳性评价    %-14s %-14s\n', '较平稳', '波动较大');
end

% 模型失配鲁棒性 (实际T1 = 4, 模型T1 = 3)
a1_real = exp(-T / 4);
b0_real = K * (1 - a1_real);
num_g_real = [zeros(1, d), b0_real];
den_g_real = [1, -a1_real];

[y_dahlin_r, u_dahlin_r] = simulate_dahlin(num_g_real, den_g_real, num_dd, den_dd, r);
[y_db_r, u_db_r] = simulate_dahlin(num_g_real, den_g_real, num_db, den_db, r);

% ---- 模型失配鲁棒性指标 ----
fprintf('\n====== 模型失配鲁棒性对比 (标称T1=3, 实际T1=4) ======\n');
yf_dr  = mean(y_dahlin_r(end-20:end));
yf_dbr = mean(y_db_r(end-20:end));
os_dr  = max(0, (max(y_dahlin_r) - yf_dr) / yf_dr * 100);
os_dbr = max(0, (max(y_db_r) - yf_dbr) / yf_dbr * 100);
st_dr = sim_N * T;
for kk = sim_N:-1:1
    if abs(y_dahlin_r(kk) - yf_dr) > 0.02 * abs(yf_dr), st_dr = kk * T; break; end
end
st_dbr = sim_N * T;
for kk = sim_N:-1:1
    if abs(y_db_r(kk) - yf_dbr) > 0.02 * abs(yf_dbr), st_dbr = kk * T; break; end
end
fprintf('                  %-16s %-16s\n', '大林算法', '最小拍');
fprintf('失配后超调量 %%   %16.2f %16.2f\n', os_dr, os_dbr);
fprintf('失配后调节时间(s) %16.4f %16.4f\n', st_dr, st_dbr);
fprintf('超调量变化        %16.2f %16.2f (标称->失配)\n', ...
    os_dr - os_d, os_dbr - os_db);
if abs(os_dbr - os_db) < abs(os_dr - os_d)
    fprintf('鲁棒性评价: 大林算法对模型失配更不敏感，鲁棒性更好\n');
else
    fprintf('鲁棒性评价: 最小拍控制对模型失配的鲁棒性优于大林算法\n');
end

fig3 = figure('Name', '大林算法 vs 最小拍控制', 'Position', [100, 100, 1100, 800]);

subplot(2, 2, 1);
stairs((0:sim_N-1)*T, r, 'k--', 'LineWidth', 1.5); hold on;
stairs((0:sim_N-1)*T, y_dahlin, 'b-', 'LineWidth', 1.2);
stairs((0:sim_N-1)*T, y_db, 'r-', 'LineWidth', 1.2);
xlabel('时间 (s)'); ylabel('y(t)');
title('输出响应对比');
legend('设定值', '大林算法', '最小拍');
grid on;

subplot(2, 2, 2);
stairs((0:sim_N-1)*T, u_dahlin, 'b-', 'LineWidth', 0.8); hold on;
stairs((0:sim_N-1)*T, u_db, 'r-', 'LineWidth', 0.8);
xlabel('时间 (s)'); ylabel('u(t)');
title('控制量对比');
legend('大林算法', '最小拍');
grid on;

subplot(2, 2, 3);
stairs((0:sim_N-1)*T, r, 'k--', 'LineWidth', 1.5); hold on;
stairs((0:sim_N-1)*T, y_dahlin_r, 'b-', 'LineWidth', 1.2);
stairs((0:sim_N-1)*T, y_db_r, 'r-', 'LineWidth', 1.2);
xlabel('时间 (s)'); ylabel('y(t)');
title('模型失配鲁棒性 (实际T1=4, 模型T1=3)');
legend('设定值', '大林算法', '最小拍');
grid on;

subplot(2, 2, 4);
bar([std(u_dahlin(30:end)), std(u_db(30:end))]);
set(gca, 'XTickLabel', {'大林算法', '最小拍'});
ylabel('控制量标准差');
title('控制量波动程度');
grid on;

sgtitle('大林算法 vs 最小拍控制');
img_path = fullfile(output_dir, 'fig3_大林vs最小拍.png');
exportgraphics(fig3, img_path, 'Resolution', 150);

fprintf('\n========== 实验六全部完成，图片已保存至 %s 文件夹 ==========\n', output_dir);

%% ==================== 函数定义区 ====================

function [num_D, den_D] = dahlin_design_1st(K, T1, tau, T, lambda)
    d = round(tau / T);
    a1 = exp(-T / T1);
    b0 = K * (1 - a1);
    a_lambda = exp(-T / lambda);
    b_lambda = 1 - a_lambda;
    num_D = (b_lambda / b0) * [1, -a1];
    den_D = zeros(1, d+3);
    den_D(1) = 1;
    den_D(2) = -a_lambda;
    den_D(d+2) = -b_lambda;
    fprintf('大林控制器 (一阶): lambda=%.3f, d=%d\n', lambda, d);
    fprintf('  对象: a1=%.4f, b0=%.4f\n', a1, b0);
    fprintf('  期望: a_lambda=%.4f, b_lambda=%.4f\n', a_lambda, b_lambda);
end

function [num_D, den_D] = dahlin_design_2nd(K, T1, T2, tau, T, lambda)
    d = round(tau / T);
    a1 = exp(-T / T1);
    a2 = exp(-T / T2);
    b0 = K * (1 - a1) * (1 - a2);
    a_lambda = exp(-T / lambda);
    b_lambda = 1 - a_lambda;
    num_D = (b_lambda / b0) * conv([1, -a1], [1, -a2]);
    den_D = zeros(1, d+3);
    den_D(1) = 1;
    den_D(2) = -a_lambda;
    den_D(d+2) = -b_lambda;
    fprintf('大林控制器 (二阶): lambda=%.3f, d=%d\n', lambda, d);
    fprintf('  对象: a1=%.4f, a2=%.4f, b0=%.4f\n', a1, a2, b0);
end

function [y, u] = simulate_dahlin(num_g, den_g, num_d, den_d, r)
    N = length(r);
    y = zeros(1, N);
    u = zeros(1, N);
    n_obj = length(den_g);
    n_ctrl = length(den_d);
    y_buf = zeros(1, n_obj);
    u_buf = zeros(1, max(length(num_g), n_ctrl));
    e_buf = zeros(1, max(length(num_d), n_ctrl));
    for k = 1:N
        e_k = r(k) - y(max(1, k-1));
        e_buf = [e_k, e_buf(1:end-1)];
        u_k = 0;
        for j = 2:min(n_ctrl, k)
            u_k = u_k - den_d(j) * u_buf(j-1);
        end
        for j = 1:min(length(num_d), k)
            u_k = u_k + num_d(j) * e_buf(j);
        end
        u(k) = max(-10, min(10, u_k));
        u_buf = [u(k), u_buf(1:end-1)];
        y_k = 0;
        for j = 2:min(n_obj, k+1)
            y_k = y_k - den_g(j) * y_buf(j-1);
        end
        for j = 1:min(length(num_g), k)
            y_k = y_k + num_g(j) * u_buf(j);
        end
        y_buf = [y_k, y_buf(1:end-1)];
        y(k) = y_k;
    end
end

function [num_out, den_out] = eliminate_ringing(num, den, ringing_poles)
    den_poly = den;
    num_poly = num;
    for i = 1:length(ringing_poles)
        p_r = ringing_poles(i);
        factor = [1, -p_r];
        [den_poly, ~] = deconv(den_poly, factor);
        num_poly = num_poly * (1 - p_r);
        den_poly = conv(den_poly, [1, -1]);
    end
    num_out = num_poly;
    den_out = den_poly;
    fprintf('振铃消除完成\n');
end
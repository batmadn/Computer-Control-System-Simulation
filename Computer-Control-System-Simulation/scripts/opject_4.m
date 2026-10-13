%% ==================== 实验四：纯滞后系统数字Smith预估控制 ====================
% 内容：Smith预估器设计与仿真、模型失配影响分析
% 输出：保存图片到 opject_4_images/ 文件夹
% ====================================================================

clear; clc;

% 创建图片保存文件夹
output_dir = 'opject_4_images';
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

%% ==================== 步骤1：建立纯滞后被控对象模型 ====================
K = 2;          % 增益
T_const = 5;    % 时间常数
tau = 3;        % 纯滞后时间

s = tf('s');
G0 = K / (T_const * s + 1);
Gs = G0 * exp(-tau * s);

T_sample = 0.1;
d = round(tau / T_sample);

G0z = c2d(G0, T_sample, 'zoh');
[num0, den0] = tfdata(G0z, 'v');

fprintf('纯滞后步数 d = %d\n', d);
fprintf('采样周期 T = %.2f s\n', T_sample);
fprintf('离散模型 G0(z): '); disp(G0z);

%% ==================== 步骤2：设计PI控制器 ====================
lambda = 1.0;

Ti = T_const;
Kp = T_const / (K * lambda);
Ki = Kp / Ti;

fprintf('\nPI控制器参数 (IMC整定):\n');
fprintf('  Kp = %.4f, Ki = %.4f, Ti = %.4f\n', Kp, Ki, Ti);
fprintf('  期望闭环时间常数 lambda = %.2f s\n', lambda);

D_cont = Kp * (1 + 1/(Ti * s));
Dz = c2d(D_cont, T_sample, 'tustin');
fprintf('离散化PI控制器 D(z): '); disp(tf(Dz));

%% ==================== 步骤3：Smith预估器 vs 普通PI控制仿真 ====================
K = 2; T_const = 5; tau = 3;
T = 0.05;
d = round(tau / T);

lambda = 1.0;
Kp = T_const / (K * lambda);
Ti = T_const;
Ki = Kp / Ti;

a = exp(-T / T_const);
b = K * (1 - a);

sim_time = 80;
N = round(sim_time / T);

r = [zeros(1, round(5/T)), ones(1, N-round(5/T))];

y_smith = zeros(1, N);
y_pi = zeros(1, N);
u_smith = zeros(1, N);
u_pi = zeros(1, N);

y0_model = 0;
y_model_buf = zeros(1, d+1);

e_sum_s = 0; e_prev_s = 0;
e_sum_p = 0; e_prev_p = 0;

x_smith = 0;  x_pi = 0;
y_obj_buf_s = zeros(1, d+1);
y_obj_buf_p = zeros(1, d+1);

for k = 2:N
    % ====== Smith预估控制 ======
    y_fb_smith = y_smith(k-1) + y0_model - y_model_buf(1);
    
    e_s = r(k) - y_fb_smith;
    e_sum_s = e_sum_s + e_s;
    u_s = Kp * e_s + Ki * T * e_sum_s;
    u_smith(k) = max(-5, min(5, u_s));
    
    y0_model = a * y0_model + b * u_smith(max(1, k-1));
    y_model_buf = [y_model_buf(2:end), y0_model];
    
    x_smith = a * x_smith + b * u_smith(max(1, k-1));
    y_obj_buf_s = [y_obj_buf_s(2:end), x_smith];
    y_smith(k) = y_obj_buf_s(1);
    
    % ====== 普通PI控制 ======
    e_p = r(k) - y_pi(k-1);
    e_sum_p = e_sum_p + e_p;
    u_p = Kp * e_p + Ki * T * e_sum_p;
    u_pi(k) = max(-5, min(5, u_p));
    
    x_pi = a * x_pi + b * u_pi(max(1, k-1));
    y_obj_buf_p = [y_obj_buf_p(2:end), x_pi];
    y_pi(k) = y_obj_buf_p(1);
end

idx_eval = round(10/T);

perf_smith = calculate_performance(y_smith, r, T, idx_eval);
perf_pi = calculate_performance(y_pi, r, T, idx_eval);

% ---- Smith vs PI 详细指标对比 ----
fprintf('\n====== Smith预估器 vs 普通PI控制 详细指标对比 ======\n');
fprintf('%-20s %-18s %-18s\n', '指标', 'Smith预估+PI', '普通PI');
fprintf('%-20s %-18s %-18s\n', '------', '------', '------');
fprintf('%-20s %-18.2f %-18.2f\n', '超调量(%%)', perf_smith.overshoot, perf_pi.overshoot);
fprintf('%-20s %-18.4f %-18.4f\n', '调节时间(s)', perf_smith.settling_time, perf_pi.settling_time);
fprintf('%-20s %-18.6f %-18.6f\n', '稳态误差', perf_smith.ess, perf_pi.ess);
fprintf('%-20s %-18.4f %-18.4f\n', 'IAE', perf_smith.IAE, perf_pi.IAE);
u_ss_s = u_smith(idx_eval:end);  u_ss_p = u_pi(idx_eval:end);
fprintf('%-20s %-18.4f %-18.4f\n', 'u_max', max(u_smith), max(u_pi));
fprintf('%-20s %-18s %-18s\n', 'u幅值范围(稳态)', ...
    sprintf('[%.2f,%.2f]', min(u_ss_s), max(u_ss_s)), ...
    sprintf('[%.2f,%.2f]', min(u_ss_p), max(u_ss_p)));
fprintf('%-20s %-18.4f %-18.4f\n', 'u_std(稳态)', std(u_ss_s), std(u_ss_p));

fprintf('\n====== Smith预估器 vs 普通PI控制 分析评价 ======\n');
fprintf('Smith预估器超调量 %.2f%%, 普通PI超调量 %.2f%%\n', perf_smith.overshoot, perf_pi.overshoot);
fprintf('Smith预估器调节时间 %.2fs, 普通PI调节时间 %.2fs\n', perf_smith.settling_time, perf_pi.settling_time);
fprintf('结论：Smith预估器通过模型预测补偿了纯滞后影响，\n');
fprintf('      在响应速度和稳定性方面均优于普通PI控制。\n');

% ========== 绘图 ==========
t = (0:N-1) * T;
fig1 = figure('Name', 'Smith预估器 vs 普通PI控制', 'Position', [100, 100, 1000, 800]);

subplot(3, 1, 1);
plot(t, r, 'k--', 'LineWidth', 1.5); hold on;
plot(t, y_smith, 'b-', 'LineWidth', 1.5);
plot(t, y_pi, 'r-', 'LineWidth', 1.5);
xlabel('时间 t (s)'); ylabel('输出 y(t)');
title('系统输出对比');
legend('设定值', 'Smith预估+PI', '普通PI');
grid on;

subplot(3, 1, 2);
stairs(t, u_smith, 'b-', 'LineWidth', 1); hold on;
stairs(t, u_pi, 'r-', 'LineWidth', 1);
xlabel('时间 t (s)'); ylabel('控制量 u(t)');
title('控制量对比');
legend('Smith预估+PI', '普通PI');
grid on;

subplot(3, 1, 3);
e_smith = r - y_smith;
e_pi = r - y_pi;
plot(t, e_smith, 'b-', 'LineWidth', 1); hold on;
plot(t, e_pi, 'r-', 'LineWidth', 1);
xlabel('时间 t (s)'); ylabel('误差 e(t)');
title('跟踪误差对比');
legend('Smith预估+PI', '普通PI');
grid on;

img_path = fullfile(output_dir, 'fig1_Smith预估器vsPI控制.png');
exportgraphics(fig1, img_path, 'Resolution', 150);

%% ==================== 步骤4：模型失配影响分析 ====================
fprintf('\n====== 模型失配影响分析 ======\n');

mismatch_scenarios = {
    '精确匹配',       1.0, 1.0, 1.0;
    '增益失配(+50%%)', 1.5, 1.0, 1.0;
    '时间常数失配(-30%%)', 1.0, 0.7, 1.0;
    '滞后失配(+30%%)', 1.0, 1.0, 1.3;
};

num_scenarios = size(mismatch_scenarios, 1);
fig2 = figure('Name', '模型失配对Smith预估器的影响', 'Position', [100, 100, 1100, 800]);

perf_mis_summary = cell(num_scenarios, 5);

for scenario = 1:num_scenarios
    scenario_label = mismatch_scenarios{scenario, 1};
    
    K_real = K * mismatch_scenarios{scenario, 2};
    T_real = T_const * mismatch_scenarios{scenario, 3};
    tau_real = tau * mismatch_scenarios{scenario, 4};
    
    a_real = exp(-T / T_real);
    b_real = K_real * (1 - a_real);
    d_real = round(tau_real / T);
    
    y0_model_mis = 0;
    y_model_buf_mis = zeros(1, d+1);
    
    x_smith_mis = 0;
    y_obj_buf_mis = zeros(1, d_real+1);
    
    e_sum_mis = 0;
    y_mis = zeros(1, N);
    u_mis = zeros(1, N);
    
    for k = 2:N
        y_fb = y_mis(k-1) + y0_model_mis - y_model_buf_mis(1);
        e_k = r(k) - y_fb;
        e_sum_mis = e_sum_mis + e_k;
        u_k = Kp * e_k + Ki * T * e_sum_mis;
        u_mis(k) = max(-5, min(5, u_k));
        
        y0_model_mis = a * y0_model_mis + b * u_mis(max(1, k-1));
        y_model_buf_mis = [y_model_buf_mis(2:end), y0_model_mis];
        
        x_smith_mis = a_real * x_smith_mis + b_real * u_mis(max(1, k-1));
        y_obj_buf_mis = [y_obj_buf_mis(2:end), x_smith_mis];
        y_mis(k) = y_obj_buf_mis(1);
    end
    
    subplot(2, 2, scenario);
    plot(t, r, 'k--', 'LineWidth', 1); hold on;
    plot(t, y_mis, 'b-', 'LineWidth', 1.5);
    xlabel('时间 t (s)'); ylabel('输出 y(t)');
    title(scenario_label);
    legend('设定值', '实际输出', 'Location', 'best');
    grid on;
    
    perf_mis = calculate_performance(y_mis, r, T, idx_eval);
    u_ss_mis = u_mis(idx_eval:end);
    perf_mis_summary{scenario, 1} = scenario_label;
    perf_mis_summary{scenario, 2} = perf_mis.overshoot;
    perf_mis_summary{scenario, 3} = perf_mis.settling_time;
    perf_mis_summary{scenario, 4} = perf_mis.ess;
    perf_mis_summary{scenario, 5} = perf_mis.IAE;
    
    fprintf('【%s】 超调=%.2f%%, 调节时间=%.2fs, 稳态误差=%.4f, IAE=%.4f, u_std=%.4f\n', ...
        scenario_label, perf_mis.overshoot, perf_mis.settling_time, ...
        perf_mis.ess, perf_mis.IAE, std(u_ss_mis));
end

sgtitle('模型失配对Smith预估控制系统性能的影响');
img_path = fullfile(output_dir, 'fig2_模型失配影响.png');
exportgraphics(fig2, img_path, 'Resolution', 150);

% ---- 模型失配汇总与分析 ----
fprintf('\n====== 模型失配影响汇总 ======\n');
fprintf('%-20s %-10s %-12s %-12s %-12s\n', ...
    '失配类型', '超调%%', '调节时间(s)', '稳态误差', 'IAE');
fprintf('%-20s %-10s %-12s %-12s %-12s\n', ...
    '------', '------', '------', '------', '------');
for i = 1:num_scenarios
    fprintf('%-20s %-10.2f %-12.4f %-12.6f %-12.4f\n', perf_mis_summary{i,:});
end

fprintf('\n====== 模型失配影响评价 ======\n');
fprintf('精确匹配时：Smith预估器性能最好，超调和IAE最小。\n');
fprintf('增益失配(+50%%)：实际增益大于模型，控制器输出偏大，超调增大。\n');
fprintf('时间常数失配(-30%%)：实际对象响应更快，模型预测滞后。\n');
fprintf('滞后失配(+30%%)：实际滞后大于模型，补偿不充分，稳定裕度降低。\n');
fprintf('结论：Smith预估器对模型精度有一定依赖，增益失配影响最大。\n');
fprintf('      实际使用中应尽可能精确辨识对象模型参数。\n');

%% ==================== 步骤5：lambda参数对Smith预估器性能的影响 ====================
fprintf('\n====== lambda参数对系统性能的影响 ======\n');

lambda_list = [0.5, 1.0, 1.5, 2.0, 3.0, 5.0];
a = exp(-T / T_const); b = K * (1 - a);

N = round(80 / T);
r = [zeros(1, round(5/T)), ones(1, N-round(5/T))];

fig3 = figure('Name', 'lambda参数对Smith预估器的影响', 'Position', [100, 100, 1200, 800]);

lambda_summary = cell(length(lambda_list), 5);

for idx = 1:length(lambda_list)
    lam = lambda_list(idx);
    
    Kp_lam = T_const / (K * lam);
    Ti_lam = T_const;
    Ki_lam = Kp_lam / Ti_lam;
    
    y0_m = 0;  y_m_buf = zeros(1, d+1);
    x_obj = 0; y_obj_buf = zeros(1, d+1);
    e_sum = 0; y_out = zeros(1, N); u_out = zeros(1, N);
    
    for k = 2:N
        y_fb = y_out(k-1) + y0_m - y_m_buf(1);
        e_k = r(k) - y_fb;
        e_sum = e_sum + e_k;
        u_out(k) = max(-5, min(5, Kp_lam*e_k + Ki_lam*T*e_sum));
        y0_m = a*y0_m + b*u_out(max(1,k-1));
        y_m_buf = [y_m_buf(2:end), y0_m];
        x_obj = a*x_obj + b*u_out(max(1,k-1));
        y_obj_buf = [y_obj_buf(2:end), x_obj];
        y_out(k) = y_obj_buf(1);
    end
    
    subplot(3, 2, idx);
    yyaxis left;
    plot(t(1:N), y_out, 'b-', 'LineWidth', 1.2); hold on;
    plot(t(1:N), r, 'k--', 'LineWidth', 1);
    ylabel('y(t)'); ylim([0 1.4]);
    yyaxis right;
    stairs(t(1:N), u_out, 'r-', 'LineWidth', 0.8);
    ylabel('u(t)');
    xlabel('时间 (s)'); title(['\lambda=' num2str(lam)]);
    grid on;
    
    perf = calculate_performance(y_out, r, T, idx_eval);
    u_ss_lam = u_out(idx_eval:end);
    lambda_summary{idx, 1} = lam;
    lambda_summary{idx, 2} = perf.overshoot;
    lambda_summary{idx, 3} = perf.settling_time;
    lambda_summary{idx, 4} = perf.IAE;
    lambda_summary{idx, 5} = std(u_ss_lam);
    fprintf('lambda=%.1f: 超调=%.2f%%, Ts=%.2fs, IAE=%.4f, u_std=%.4f\n', ...
        lam, perf.overshoot, perf.settling_time, perf.IAE, std(u_ss_lam));
end

sgtitle('\lambda参数对Smith预估系统性能的影响');
img_path = fullfile(output_dir, 'fig3_lambda参数影响.png');
exportgraphics(fig3, img_path, 'Resolution', 150);

% ---- lambda 参数影响分析 ----
fprintf('\n====== lambda参数影响汇总 ======\n');
fprintf('%-8s %-10s %-12s %-12s %-12s\n', 'lambda', '超调%%', '调节时间(s)', 'IAE', 'u_std');
fprintf('%-8s %-10s %-12s %-12s %-12s\n', '------', '------', '------', '------', '------');
for i = 1:length(lambda_list)
    fprintf('%-8.1f %-10.2f %-12.4f %-12.4f %-12.4f\n', lambda_summary{i,:});
end
fprintf('\n结论：lambda增大→响应变慢但控制量更平稳，超调减小。\n');
fprintf('      lambda减小→响应变快但控制量波动大，可能有超调。\n');
fprintf('      本系统推荐lambda=%.1f~%.1f时综合性能较好。\n', T_const/3, T_const);

fprintf('\n========== 实验四全部完成，图片已保存至 %s 文件夹 ==========\n', output_dir);

%% ==================== 函数定义区 ====================

function perf = calculate_performance(y, r, T, idx_start)
    if nargin < 4
        idx_start = 1;
    end
    
    y_eval = y(idx_start:end);
    r_eval = r(idx_start:end);
    
    y_final = mean(y_eval(end-round(1/T):end));
    if abs(y_final) < 1e-10
        y_final = 1e-10;
    end
    
    perf.ess = abs(r_eval(1) - y_final);
    
    y_max = max(y_eval);
    perf.overshoot = max(0, (y_max - y_final) / abs(y_final) * 100);
    
    perf.settling_time = length(y) * T;
    for k = length(y):-1:idx_start
        if abs(y(k) - y_final) > 0.02 * abs(y_final)
            perf.settling_time = k * T;
            break;
        end
    end
    
    perf.IAE = sum(abs(r_eval - y_eval)) * T;
end
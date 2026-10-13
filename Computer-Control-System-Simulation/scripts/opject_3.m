%% ==================== 实验三：数字PID控制器设计与仿真 ====================
% 内容：数字PID控制器、Ziegler-Nichols整定、不同参数对比、离散化方法对比
% 输出：保存图片到 opject_3_images/ 文件夹
% ====================================================================

clear; clc;

% 创建图片保存文件夹
output_dir = 'opject_3_images';
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

%% ==================== 步骤1：建立被控对象模型 ====================
% 被控对象: 直流电机模型 G(s) = 10 / (s(s+2)(s+5))
num = [10];
den = conv([1 2], [1 5 0]);  % = [1 7 10 0]
Gs = tf(num, den);

% 设计参数
T = 0.01;  % 采样周期 10ms
sim_time = 10;

% 离散化被控对象
Gz = c2d(Gs, T, 'zoh');
[numz, denz] = tfdata(Gz, 'v');

fprintf('被控对象 G(s) = 10 / (s(s+2)(s+5))\n');
fprintf('采样周期 T = %.2f s\n', T);
fprintf('离散化后 G(z) 极点: '); disp(pole(Gz)');

%% ==================== 步骤2：Ziegler-Nichols法整定PID参数 ====================
fprintf('\n====== Z-N法PID参数整定 ======\n');

[Gm, Pm, Wcg, Wcp] = margin(Gs);
Ku = Gm;
Tu = 2 * pi / Wcg;

fprintf('临界增益 Ku = %.4f\n', Ku);
fprintf('临界周期 Tu = %.4f s\n', Tu);
fprintf('增益裕度 = %.2f dB, 相位穿越频率 = %.4f rad/s\n', 20*log10(Gm), Wcg);

% 验证：纯比例控制下的振荡
fig_osc = figure('Name', 'ZN法整定——临界振荡验证', 'Position', [100, 100, 800, 400]);
Gc_osc = feedback(Ku * Gs, 1);
step(Gc_osc, 50);
title(['临界振荡响应 (Ku=' num2str(Ku, '%.2f') ')']);
grid on;
img_path = fullfile(output_dir, 'fig1_临界振荡验证.png');
exportgraphics(fig_osc, img_path, 'Resolution', 150);

% 根据ZN法计算PID参数
fprintf('\nZ-N法推荐参数:\n');
fprintf('----------------|----------|----------|----------\n');
fprintf('  控制器类型     |    Kp    |    Ti    |    Td    \n');
fprintf('----------------|----------|----------|----------\n');
fprintf('  P 控制器       | %8.3f |    --    |    --    \n', 0.5*Ku);
fprintf('  PI 控制器      | %8.3f | %8.3f |    --    \n', 0.45*Ku, 0.83*Tu);
fprintf('  PID 控制器     | %8.3f | %8.3f | %8.3f \n', 0.6*Ku, 0.5*Tu, 0.125*Tu);
fprintf('----------------|----------|----------|----------\n');

Kp_zn = 0.6 * Ku;
Ti_zn = 0.5 * Tu;
Td_zn = 0.125 * Tu;
Ki_zn = Kp_zn / Ti_zn;
Kd_zn = Kp_zn * Td_zn;

fprintf('\nZN法PID参数: Kp=%.4f, Ki=%.4f, Kd=%.4f\n', Kp_zn, Ki_zn, Kd_zn);

%% ==================== 步骤3：仿真对比不同PID参数下的系统性能 ====================
fprintf('\n====== 不同PID参数下的阶跃响应对比 ======\n');

% PID参数组 [Kp, Ki, Kd, 标签]
pid_params = {
    1.0, 0.5, 0.1, '保守参数';
    Kp_zn, Ki_zn, Kd_zn, 'ZN法参数';
    5.0, 5.0, 1.0, '激进参数';
    3.0, 1.0, 2.0, '大微分参数';
};

n_samples = round(sim_time / T);
t_axis = (0:n_samples-1) * T;
r = [zeros(1, round(1/T)), ones(1, n_samples - round(1/T))];  % 1秒后阶跃

fig_pid = figure('Name', '不同PID参数下的阶跃响应对比', 'Position', [100, 100, 1100, 800]);

perf_summary = cell(size(pid_params,1), 6);

for pid_idx = 1:size(pid_params, 1)
    Kp = pid_params{pid_idx, 1};
    Ki = pid_params{pid_idx, 2};
    Kd = pid_params{pid_idx, 3};
    label_str = pid_params{pid_idx, 4};
    
    y = zeros(1, n_samples);
    u = zeros(1, n_samples);
    
    u_prev = 0;
    e_hist = [0, 0];
    
    for k = 3:n_samples
        e_k = r(k) - y(k-1);
        
        delta_u = Kp * (e_k - e_hist(1)) + ...
                  Ki * T * e_k + ...
                  (Kd / T) * (e_k - 2*e_hist(1) + e_hist(2));
        
        u(k) = u_prev + delta_u;
        u(k) = max(-10, min(10, u(k)));
        
        y(k) = 0;
        for j = 2:length(denz)
            if k-j+1 > 0
                y(k) = y(k) - denz(j) * y(k-j+1);
            end
        end
        for j = 1:length(numz)
            if k-j > 0
                y(k) = y(k) + numz(j) * u(k-j);
            end
        end
        
        u_prev = u(k);
        e_hist = [e_k, e_hist(1)];
    end
    
    subplot(2, 2, pid_idx);
    
    yyaxis left;
    plot(t_axis, y, 'b-', 'LineWidth', 1.5);
    hold on;
    plot(t_axis, r, 'r--', 'LineWidth', 1);
    ylabel('输出 y(t)');
    ylim([0 1.8]);
    
    yyaxis right;
    stairs(t_axis, u, 'g-', 'LineWidth', 1);
    ylabel('控制量 u(t)');
    
    xlabel('时间 (s)');
    title(['PID响应: ' label_str]);
    legend('y(t)', 'r(t)', 'u(t)', 'Location', 'best');
    grid on;
    
    idx_start = round(2/T);
    perf = calculate_performance(y(idx_start:end), r(idx_start:end), T);
    
    u_ss = u(idx_start:end);
    u_max_val = max(u);
    
    fprintf('【%s】\n', label_str);
    fprintf('  超调量: %.2f%%, 调节时间: %.3fs, 稳态误差: %.4f, IAE: %.4f\n', ...
        perf.overshoot, perf.settling_time, perf.ess, perf.IAE);
    fprintf('  u_max=%.4f, u_幅值范围(稳态)=[%.2f,%.2f], u_std(稳态)=%.4f\n', ...
        u_max_val, min(u_ss), max(u_ss), std(u_ss));
    
    perf_summary{pid_idx, 1} = label_str;
    perf_summary{pid_idx, 2} = perf.overshoot;
    perf_summary{pid_idx, 3} = perf.settling_time;
    perf_summary{pid_idx, 4} = perf.ess;
    perf_summary{pid_idx, 5} = perf.IAE;
    perf_summary{pid_idx, 6} = std(u_ss);
end

sgtitle('不同PID参数下的阶跃响应对比');
img_path = fullfile(output_dir, 'fig2_不同PID参数对比.png');
exportgraphics(fig_pid, img_path, 'Resolution', 150);

% ---- PID参数对比汇总与分析 ----
fprintf('\n====== PID参数对比汇总 ======\n');
fprintf('%-16s %-10s %-12s %-12s %-12s %-12s\n', ...
    '参数组', '超调%%', '调节时间(s)', '稳态误差', 'IAE', 'u_std');
fprintf('%-16s %-10s %-12s %-12s %-12s %-12s\n', ...
    '------', '------', '------', '------', '------', '------');
for i = 1:size(perf_summary,1)
    fprintf('%-16s %-10.2f %-12.4f %-12.6f %-12.4f %-12.4f\n', perf_summary{i,:});
end

fprintf('\n====== PID参数影响分析 ======\n');
fprintf('保守参数：超调量小、响应平稳但调节时间较长，u_std最小，控制平稳。\n');
fprintf('ZN法参数：超调量适中，综合性能较好，是工程常用的初始整定方法。\n');
fprintf('激进参数：上升快但超调量大，u波动剧烈（u_std最大），可能振荡甚至不稳定。\n');
fprintf('大微分参数：微分作用强可抑制超调，但引入了高频噪声敏感风险。\n');
fprintf('结论：PID参数需在快速性与稳定性之间折中。ZN法提供良好起点，\n');
fprintf('      实际工程中常在ZN法基础上微调，以平衡各项指标。\n');

%% ==================== 步骤4：对比不同离散化方法 ====================
s = tf('s');
D_cont = 2 + 1/s + 0.5*s;  % 连续PID: Kp=2, Ki=1, Kd=0.5

T_disc = 0.1;

% 手动构造后向差分和前向差分（c2d 不支持 'backward'/'forward'）
z = tf('z', T_disc);
s_back = (z - 1) / (T_disc * z);   % 后向差分: s=(z-1)/(T*z)
s_fwd  = (z - 1) / T_disc;          % 前向差分: s=(z-1)/T

Kp_d = 2; Ki_d = 1; Kd_d = 0.5;
Dz_backward = Kp_d + Ki_d/s_back + Kd_d*s_back;
Dz_forward  = Kp_d + Ki_d/s_fwd  + Kd_d*s_fwd;
Dz_tustin   = c2d(D_cont, T_disc, 'tustin');

% 化简
Dz_backward = minreal(Dz_backward);
Dz_forward  = minreal(Dz_forward);

fig_disc = figure('Name', 'PID离散化方法对比', 'Position', [100, 100, 1000, 600]);
bode(D_cont, 'b-', Dz_backward, 'r--', Dz_forward, 'g-.', Dz_tustin, 'm:');
legend('连续PID', '后向差分', '前向差分', '双线性变换');
title('不同离散化方法的频率响应对比');
grid on;

img_path = fullfile(output_dir, 'fig3_离散化方法对比.png');
exportgraphics(fig_disc, img_path, 'Resolution', 150);

% 打印各离散化方法的传递函数
fprintf('\n====== 离散化方法对比 ======\n');
fprintf('连续PID: D(s) = 2 + 1/s + 0.5s\n');
fprintf('后向差分 D(z) = '); disp(tf(Dz_backward));
fprintf('前向差分 D(z) = '); disp(tf(Dz_forward));
fprintf('双线性变换 D(z) = '); disp(tf(Dz_tustin));

% ---- 闭环阶跃响应对比 ----
% 对步骤1的电机对象 Gz 重新离散化到 T_disc=0.1 的控制器配合
Gz_disc = c2d(Gs, T_disc, 'zoh');
[numz_disc, denz_disc] = tfdata(Gz_disc, 'v');

[num_d_bk, den_d_bk] = tfdata(Dz_backward, 'v');
[num_d_fw, den_d_fw] = tfdata(Dz_forward,  'v');
[num_d_tt, den_d_tt] = tfdata(Dz_tustin,   'v');

n_sim = round(5 / T_disc);
r_sim = ones(1, n_sim);
[y_bk, ~] = simulate_deadbeat_disc(numz_disc, denz_disc, num_d_bk, den_d_bk, r_sim);
[y_fw, ~] = simulate_deadbeat_disc(numz_disc, denz_disc, num_d_fw, den_d_fw, r_sim);
[y_tt, ~] = simulate_deadbeat_disc(numz_disc, denz_disc, num_d_tt, den_d_tt, r_sim);

% 计算阶跃响应指标
yf_bk = mean(y_bk(end-round(1/T_disc):end));
yf_fw = mean(y_fw(end-round(1/T_disc):end));
yf_tt = mean(y_tt(end-round(1/T_disc):end));
os_bk  = max(0, (max(y_bk) - yf_bk) / abs(yf_bk) * 100);
os_fw  = max(0, (max(y_fw) - yf_fw) / abs(yf_fw) * 100);
os_tt  = max(0, (max(y_tt) - yf_tt) / abs(yf_tt) * 100);

ts_bk = n_sim * T_disc;
for kk = n_sim:-1:1
    if abs(y_bk(kk) - yf_bk) > 0.02 * abs(yf_bk), ts_bk = kk * T_disc; break; end
end
ts_fw = n_sim * T_disc;
for kk = n_sim:-1:1
    if abs(y_fw(kk) - yf_fw) > 0.02 * abs(yf_fw), ts_fw = kk * T_disc; break; end
end
ts_tt = n_sim * T_disc;
for kk = n_sim:-1:1
    if abs(y_tt(kk) - yf_tt) > 0.02 * abs(yf_tt), ts_tt = kk * T_disc; break; end
end

fprintf('\n====== 各离散化方法闭环阶跃响应指标 ======\n');
fprintf('%-16s %-14s %-14s %-14s\n', '指标', '后向差分', '前向差分', '双线性变换');
fprintf('%-16s %-14s %-14s %-14s\n', '------', '------', '------', '------');
fprintf('%-16s %-14.2f %-14.2f %-14.2f\n', '超调量 %%', os_bk, os_fw, os_tt);
fprintf('%-16s %-14.4f %-14.4f %-14.4f\n', '调节时间 (s)', ts_bk, ts_fw, ts_tt);
fprintf('%-16s %-14.6f %-14.6f %-14.6f\n', '稳态值', yf_bk, yf_fw, yf_tt);

fprintf('\n评价：\n');
fprintf('双线性变换(Tustin)在保持频率特性方面精度最高，\n');
fprintf('  是工程中最常用的离散化方法，阶跃响应最接近连续系统。\n');
fprintf('后向差分稳定性好但高频相位滞后较大，响应偏慢。\n');
fprintf('前向差分可能引起不稳定（离散极点可能移出单位圆），高频特性差。\n');

%% ==================== 步骤5：PID参数对系统性能的影响分析 ====================
fprintf('\n====== Kp参数变化的影响 ======\n');

Kp_list = [0.5, 1.0, 2.0, Kp_zn, 5.0, 8.0];
fig_kp = figure('Name', 'Kp参数变化的影响', 'Position', [100, 100, 1200, 400]);

y_vals_kp = cell(length(Kp_list), 1);

for i = 1:length(Kp_list)
    Kp_t = Kp_list(i);
    Ki_t = Ki_zn;
    Kd_t = Kd_zn;
    
    y = zeros(1, n_samples);
    u = zeros(1, n_samples);
    u_prev = 0;
    e_hist = [0, 0];
    
    for k = 3:n_samples
        e_k = r(k) - y(k-1);
        delta_u = Kp_t * (e_k - e_hist(1)) + ...
                  Ki_t * T * e_k + ...
                  (Kd_t / T) * (e_k - 2*e_hist(1) + e_hist(2));
        u(k) = max(-10, min(10, u_prev + delta_u));
        
        y(k) = 0;
        for j = 2:length(denz)
            if k-j+1 > 0
                y(k) = y(k) - denz(j) * y(k-j+1);
            end
        end
        for j = 1:length(numz)
            if k-j > 0
                y(k) = y(k) + numz(j) * u(k-j);
            end
        end
        u_prev = u(k);
        e_hist = [e_k, e_hist(1)];
    end
    
    plot(t_axis, y, 'LineWidth', 1.2); hold on;
    y_vals_kp{i} = y;
end

hold off;
xlabel('时间 (s)'); ylabel('输出 y(t)');
title('Kp参数变化对系统响应的影响');
legend(arrayfun(@(x) sprintf('Kp=%.1f', x), Kp_list, 'UniformOutput', false));
grid on;

img_path = fullfile(output_dir, 'fig4_Kp参数影响.png');
exportgraphics(fig_kp, img_path, 'Resolution', 150);

% ---- Kp 参数影响数值分析 ----
fprintf('\n====== Kp参数影响数值汇总 ======\n');
fprintf('%-8s %-10s %-12s %-12s\n', 'Kp', '超调%%', '调节时间(s)', '稳态误差');
fprintf('%-8s %-10s %-12s %-12s\n', '------', '------', '------', '------');
idx_kp_start = round(2/T);
for i = 1:length(Kp_list)
    perf_kp = calculate_performance(y_vals_kp{i}(idx_kp_start:end), r(idx_kp_start:end), T);
    fprintf('%-8.1f %-10.2f %-12.4f %-12.6f\n', Kp_list(i), perf_kp.overshoot, perf_kp.settling_time, perf_kp.ess);
end
fprintf('\n结论：Kp增大 → 响应变快但超调增大，系统趋向振荡。\n');
fprintf('      Kp过大可能导致系统不稳定。Kp减小 → 响应平稳但跟踪慢。\n');
fprintf('      ZN法推荐Kp=%.2f 在快速性和稳定性之间取得较好平衡。\n', Kp_zn);

fprintf('\n========== 实验三全部完成，图片已保存至 %s 文件夹 ==========\n', output_dir);

%% ==================== 函数定义区 ====================

function perf = calculate_performance(y, r, T_val)
    y_final = mean(y(end-round(1/T_val):end));
    if abs(y_final) < 1e-10
        y_final = 1e-10;
    end
    perf.ess = abs(r(1) - y_final);
    y_max = max(y);
    perf.overshoot = max(0, (y_max - y_final) / abs(y_final) * 100);
    perf.settling_time = length(y) * T_val;
    for k = length(y):-1:1
        if abs(y(k) - y_final) > 0.02 * abs(y_final)
            perf.settling_time = k * T_val;
            break;
        end
    end
    perf.IAE = sum(abs(r - y)) * T_val;
end

function [y, u] = simulate_deadbeat_disc(numz, denz, num_d, den_d, r)
    % 与 simulate_deadbeat 相同的闭环仿真（用于离散化方法对比）
    N = length(r);
    y = zeros(1, N);
    u = zeros(1, N);
    n_ctrl_num = length(num_d);
    n_ctrl_den = length(den_d);
    n_obj_num = length(numz);
    n_obj_den = length(denz);
    u_buf = zeros(1, max([n_ctrl_den, n_ctrl_num, n_obj_num]));
    e_buf = zeros(1, max([n_ctrl_num, n_ctrl_den]));
    y_buf = zeros(1, n_obj_den);
    for k = 1:N
        e_k = r(k) - y(max(1, k-1));
        e_buf = [e_k, e_buf(1:end-1)];
        u_temp = 0;
        for j = 2:min(n_ctrl_den, k)
            u_temp = u_temp - den_d(j) * u_buf(j-1);
        end
        for j = 1:min(n_ctrl_num, k)
            u_temp = u_temp + num_d(j) * e_buf(j);
        end
        u_buf = [u_temp, u_buf(1:end-1)];
        u(k) = u_temp;
        y_temp = 0;
        for j = 2:min(n_obj_den, k+1)
            y_temp = y_temp - denz(j) * y_buf(j-1);
        end
        for j = 1:min(n_obj_num, k)
            y_temp = y_temp + numz(j) * u_buf(j);
        end
        y_buf = [y_temp, y_buf(1:end-1)];
        y(k) = y_temp;
    end
end
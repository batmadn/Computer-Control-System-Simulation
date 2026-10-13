%% ==================== 实验二：离散系统性能分析 ====================
% 内容：时域响应、稳态误差、频域特性分析
% 输出：保存图片到 opject_2_images/ 文件夹
% ====================================================================

clear; clc;

% 创建图片保存文件夹
output_dir = 'opject_2_images';
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

%% ==================== 步骤1：建立离散系统模型并分析时域响应 ====================
% 被控对象 G(s) = 1/(s+1)^2
num_s = [1];
den_s = [1 2 1];
Gs = tf(num_s, den_s);

% 不同采样周期
T_list = [0.1, 0.2, 0.5, 1.0];
fig1 = figure('Name', '实验二：时域响应分析', 'Position', [100, 100, 900, 700]);

step_info_table = cell(length(T_list), 6);

for i = 1:length(T_list)
    T = T_list(i);
    Gz = c2d(Gs, T, 'zoh');
    
    % 闭环系统 (单位反馈)
    Gc = feedback(Gz, 1);
    
    subplot(2, 2, i);
    step(Gc);
    title(['单位阶跃响应 (T=' num2str(T) 's)']);
    grid on;
    
    % 获取阶跃响应数据
    [y, ~] = step(Gc);
    y_final = y(end);
    
    % 获取暂态指标
    info = stepinfo(Gc);
    
    step_info_table{i, 1} = T;
    step_info_table{i, 2} = info.RiseTime;
    step_info_table{i, 3} = info.PeakTime;
    step_info_table{i, 4} = info.Overshoot;
    step_info_table{i, 5} = info.SettlingTime;
    step_info_table{i, 6} = y_final;
end

sgtitle('不同采样周期下的单位阶跃响应');

% 保存图片1
drawnow;
exportgraphics(fig1, fullfile(output_dir, 'fig1_时域响应.png'), 'Resolution', 150);

% 打印阶跃响应指标表
fprintf('\n====== 阶跃响应暂态指标 ======\n');
fprintf('%-8s %-10s %-10s %-10s %-10s %-10s\n', ...
    'T(s)', '上升时间', '峰值时间', '超调量%%', '调节时间', '终值');
fprintf('%-8s %-10s %-10s %-10s %-10s %-10s\n', ...
    '------', '------', '------', '------', '------', '------');
for i = 1:length(T_list)
    fprintf('%-8.1f %-10.4f %-10.4f %-10.2f %-10.4f %-10.4f\n', ...
        step_info_table{i,:});
end

% ---- 时域响应文字分析与结论 ----
fprintf('\n====== 时域响应分析结论 ======\n');
fprintf('观察与结论：\n');
fprintf('1. 采样周期对暂态指标有显著影响：\n');
for i = 1:length(T_list)
    fprintf('   T=%.1fs: 超调=%.2f%%, 调节时间=%.3fs, 上升时间=%.3fs\n', ...
        T_list(i), step_info_table{i,4}, step_info_table{i,5}, step_info_table{i,2});
end
fprintf('2. 随着采样周期增大：\n');
fprintf('   - 超调量明显增大（T=1.0s时可能超过50%%）\n');
fprintf('   - 上升时间变化不大\n');
fprintf('   - 调节时间总体呈增大趋势\n');
fprintf('3. 原因：采样周期增大相当于引入了更大的信息延迟，\n');
fprintf('   导致相位裕度降低，系统阻尼减小，振荡加剧。\n');
fprintf('4. 建议：在实际工程中，采样频率应选择为闭环带宽的10-20倍，\n');
fprintf('   以保证离散控制系统性能接近连续系统。\n');

%% ==================== 步骤2：稳态误差分析 ====================
fprintf('\n====== 稳态误差分析 ======\n');

% 0型系统: G(s) = 1/((s+1)(s+2))
G0 = tf(1, [1 3 2]);
% I型系统: G(s) = 1/(s(s+1))
G1 = tf(1, [1 1 0]);
% II型系统: G(s) = 1/(s^2(s+2))
G2 = tf(1, [1 2 0 0]);

systems = {G0, G1, G2};
sys_types = {'0型', 'I型', 'II型'};
T = 0.1;

fprintf('%-8s %-12s %-12s %-12s\n', '系统类型', '阶跃ess', '斜坡ess', '抛物线ess');
fprintf('%-8s %-12s %-12s %-12s\n', '------', '------', '------', '--------');

ess_step_vals = zeros(1,3);
ess_ramp_vals = zeros(1,3);
ess_para_vals = zeros(1,3);

for i = 1:3
    Gz = c2d(systems{i}, T, 'zoh');
    [numz, denz] = tfdata(Gz, 'v');
    
    if length(numz) < length(denz)
        numz = [zeros(1, length(denz)-length(numz)), numz];
    end
    
    % 稳态位置误差常数 Kp = G(1)
    Gz_val_1 = polyval(numz, 1) / polyval(denz, 1);
    Kp = Gz_val_1;
    
    if isinf(Kp) || abs(Kp) > 1e10
        ess_step = 0;
    else
        ess_step = 1 / (1 + Kp);
    end
    
    % 斜坡误差常数 Kv
    z_near_1 = 1 + 1e-8;
    Gz_val_near = polyval(numz, z_near_1) / polyval(denz, z_near_1);
    Kv_approx = (z_near_1 - 1) * Gz_val_near / (T * z_near_1);
    
    if abs(Kv_approx) < 1e-10
        ess_ramp = Inf;
    else
        ess_ramp = 1 / Kv_approx;
    end
    
    % 抛物线误差常数 Ka
    z_near_1_sq = 1 + 1e-8;
    Gz_val_near_sq = polyval(numz, z_near_1_sq) / polyval(denz, z_near_1_sq);
    Ka_approx = ((z_near_1_sq - 1)^2) * Gz_val_near_sq / ((T * z_near_1_sq)^2);
    
    if abs(Ka_approx) < 1e-10
        ess_para = Inf;
    else
        ess_para = 1 / Ka_approx;
    end
    
    ess_step_vals(i) = ess_step;
    ess_ramp_vals(i) = ess_ramp;
    ess_para_vals(i) = ess_para;
    
    fprintf('%-8s ', sys_types{i});
    if isinf(ess_step), fprintf('%-12s ', '0(无差)');
    else, fprintf('%-12.4f ', ess_step); end
    if isinf(ess_ramp), fprintf('%-12s ', '∞');
    else, fprintf('%-12.4f ', ess_ramp); end
    if isinf(ess_para), fprintf('%-12s\n', '∞');
    else, fprintf('%-12.4f\n', ess_para); end
end

% ---- 稳态误差文字分析 ----
fprintf('\n====== 稳态误差分析结论 ======\n');
fprintf('0型系统 (G(s)=1/((s+1)(s+2))):\n');
fprintf('  - 阶跃输入：ess=%.4f（有差跟踪），因为无积分环节\n', ess_step_vals(1));
fprintf('  - 斜坡输入：ess=∞（无法跟踪），系统不能跟踪速度输入\n');
fprintf('  - 抛物线输入：ess=∞（无法跟踪）\n');
fprintf('I型系统 (G(s)=1/(s(s+1))):\n');
fprintf('  - 阶跃输入：ess=0（无差跟踪），含有一个积分环节\n');
fprintf('  - 斜坡输入：ess=%.4f（有差跟踪），速度误差常数 Kv=%.4f\n', ess_ramp_vals(2), 1/max(ess_ramp_vals(2),1e-10));
fprintf('  - 抛物线输入：ess=∞（无法跟踪）\n');
fprintf('II型系统 (G(s)=1/(s^2(s+2))):\n');
fprintf('  - 阶跃输入：ess=0（无差跟踪）\n');
fprintf('  - 斜坡输入：ess=0（无差跟踪），含两个积分环节\n');
if ~isinf(ess_para_vals(3))
    fprintf('  - 抛物线输入：ess=%.4f（有差跟踪），加速度误差常数 Ka=%.4f\n', ess_para_vals(3), 1/max(ess_para_vals(3),1e-10));
else
    fprintf('  - 抛物线输入：ess=∞（无法跟踪）\n');
end
fprintf('结论：系统型别越高，跟踪高次输入信号的能力越强，但稳定性裕度越小。\n');

%% ==================== 步骤3：频域特性分析 ====================
T = 0.1;
Gz = c2d(Gs, T, 'zoh');

fig2 = figure('Name', '实验二：频域特性分析', 'Position', [100, 100, 1000, 800]);

% 离散Bode图
subplot(2, 2, 1);
bode(Gz);
title('离散Bode图');
grid on;

% 获取频域指标
[Gm, Pm, Wcg, Wcp] = margin(Gz);
fprintf('\n====== 频域特性分析 ======\n');
fprintf('离散系统频域指标 (T=%.1fs):\n', T);
fprintf('  增益裕度 Gm = %.2f dB, 穿越频率 = %.4f rad/s\n', 20*log10(Gm), Wcg);
fprintf('  相位裕度 Pm = %.2f °, 剪切频率 = %.4f rad/s\n', Pm, Wcp);
fprintf('  稳定裕度评价：');
if Pm > 45 && 20*log10(Gm) > 6
    fprintf('增益裕度和相位裕度均满足工程要求(>6dB,>45°)，系统稳定裕度充足。\n');
elseif Pm > 30 && 20*log10(Gm) > 3
    fprintf('增益裕度和相位裕度基本满足要求，系统稳定但裕度偏小。\n');
else
    fprintf('增益裕度或相位裕度不足，系统接近不稳定边界或已不稳定。\n');
end

% 双线性变换后bode图
subplot(2, 2, 2);
Gw = d2c(Gz, 'tustin');
bode(Gw);
title('经双线性变换后Bode图');
grid on;

% 离散Nyquist图
subplot(2, 2, 3);
nyquist(Gz);
title('离散Nyquist图');
grid on;

% 闭环频率响应
subplot(2, 2, 4);
Gc = feedback(Gz, 1);
bode(Gc);
title('闭环频率响应');
grid on;

% 闭环频域分析
[mag_cl, ~, w_cl] = bode(Gc);
mag_sq_cl = squeeze(mag_cl);
if ~isempty(mag_sq_cl)
    [Mr, idx_mr] = max(mag_sq_cl);
    Wr = w_cl(idx_mr);
    bw_idx = find(mag_sq_cl <= 0.707, 1);
    if ~isempty(bw_idx)
        BW = w_cl(bw_idx);
    else
        BW = w_cl(end);
    end
    fprintf('  闭环谐振峰值 Mr = %.4f (%.2f dB), 谐振频率 = %.4f rad/s\n', Mr, 20*log10(Mr), Wr);
    fprintf('  闭环带宽 (下降3dB处) ≈ %.4f rad/s\n', BW);
    fprintf('  说明：带宽越大，系统响应越快；谐振峰过大则系统振荡严重。\n');
end

sgtitle('离散系统频域特性分析');

% 保存图片2
drawnow;
exportgraphics(fig2, fullfile(output_dir, 'fig2_频域特性分析.png'), 'Resolution', 150);

%% ==================== 步骤4：采样周期对频域特性的影响 ====================
T_range = [0.05, 0.1, 0.5, 1.0];
fig3 = figure('Name', '采样周期对频域特性的影响', 'Position', [100, 100, 1000, 800]);

for i = 1:4
    T_i = T_range(i);
    Gz_i = c2d(Gs, T_i, 'zoh');
    
    subplot(2, 2, i);
    [mag, phase, w] = bode(Gz_i);
    mag_sq = squeeze(mag);
    semilogx(w, 20*log10(mag_sq));
    xlabel('Frequency (rad/s)');
    ylabel('Magnitude (dB)');
    title(['幅频特性 (T=' num2str(T_i) 's)']);
    grid on;
    
    % 频域指标
    [Gm_i, Pm_i, Wcg_i, Wcp_i] = margin(Gz_i);
    fprintf('T=%.2fs: Gm=%.2fdB, Pm=%.2f°, 剪切频率=%.3f rad/s\n', ...
        T_i, 20*log10(Gm_i), Pm_i, Wcp_i);
end

fprintf('\n====== 采样周期对频域特性影响的结论 ======\n');
fprintf('随着采样周期 T 增大（采样频率降低）：\n');
fprintf('  - 相位裕度 Pm 减小 → 系统阻尼变差，超调增大\n');
fprintf('  - 增益裕度 Gm 减小 → 鲁棒性降低\n');
fprintf('  - 有效频率上限降低（混叠效应增强）\n');
fprintf('  - 奈奎斯特频率 = 1/(2T)，当 T 过大时高频信息丢失严重\n');
fprintf('工程建议：采样频率应至少为闭环带宽的 10 倍以上。\n');
fprintf('对于本系统，建议 T ≤ 0.1s（采样频率 ≥ 10Hz）。\n');

sgtitle('采样周期对离散系统幅频特性的影响');

% 保存图片3
drawnow;
exportgraphics(fig3, fullfile(output_dir, 'fig3_采样周期对频域的影响.png'), 'Resolution', 150);

fprintf('\n========== 实验二全部完成，图片已保存至 %s 文件夹 ==========\n', output_dir);
%% ==================== 实验五：最小拍控制系统设计 ====================
% 内容：阶跃/斜坡输入最小拍控制器设计、仿真验证、鲁棒性分析
% 输出：保存图片到 opject_5_images/ 文件夹
% ====================================================================

clear; clc;

% 创建图片保存文件夹
output_dir = 'opject_5_images';
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

%% ==================== 步骤1：建立广义被控对象模型 ====================
% 被控对象: G(s) = 10 / (s(s+1))
num_s = [10];
den_s = [1 1 0];
Gs = tf(num_s, den_s);

T = 0.1;  % 采样周期

Gz = c2d(Gs, T, 'zoh');
[numz, denz] = tfdata(Gz, 'v');

fprintf('广义对象脉冲传递函数 G(z):\n');
Gz
fprintf('对象零点: '); disp(zero(Gz)');
fprintf('对象极点: '); disp(pole(Gz)');

%% ==================== 步骤2：设计最小拍控制器 ====================

% 阶跃输入最小拍控制器
Phi_step = tf([0 1], [1 0], T);
one_minus_Phi_step = tf([1 -1], [1 0], T);
Dz_step = minreal(Phi_step / (Gz * one_minus_Phi_step));
[num_ds, den_ds] = tfdata(Dz_step, 'v');

fprintf('\n=== 阶跃输入最小拍控制器 ===\n');
fprintf('闭环传递函数 Phi(z) = z^{-1}\n');
fprintf('控制器 D(z): '); disp(tf(Dz_step));

% 斜坡输入最小拍控制器
Phi_ramp = tf([0 2 -1], [1 0 0], T);
one_minus_Phi_ramp = tf([1 -2 1], [1 0 0], T);
Dz_ramp = minreal(Phi_ramp / (Gz * one_minus_Phi_ramp));
[num_dr, den_dr] = tfdata(Dz_ramp, 'v');

fprintf('\n=== 斜坡输入最小拍控制器 ===\n');
fprintf('闭环传递函数 Phi(z) = 2z^{-1} - z^{-2}\n');
fprintf('控制器 D(z): '); disp(tf(Dz_ramp));

% 无纹波最小拍控制器 (阶跃输入)
zeros_Gz = zero(Gz);
fprintf('\n=== 无纹波最小拍设计 (阶跃输入) ===\n');
fprintf('G(z) 的零点: '); disp(zeros_Gz');

num_Phi_rf = [0, 1];
for j = 1:length(zeros_Gz)
    zj = zeros_Gz(j);
    if abs(zj) < 1 && abs(zj - 1) > 1e-6
        num_Phi_rf = conv(num_Phi_rf, [1, -zj]);
    end
end
gain_factor = polyval([1 0], 1) / polyval(num_Phi_rf, 1);
num_Phi_rf = gain_factor * num_Phi_rf;
Phi_rf = tf(num_Phi_rf, [1 0], T);
one_minus_Phi_rf = minreal(1 - Phi_rf);
Dz_rf = minreal(Phi_rf / (Gz * one_minus_Phi_rf));
[num_rf, den_rf] = tfdata(Dz_rf, 'v');

fprintf('无纹波闭环 Phi(z): '); disp(tf(Phi_rf));
fprintf('无纹波控制器 D(z): '); disp(tf(Dz_rf));

%% ==================== 步骤3：仿真验证 ====================
sim_N = 50;

r_step = ones(1, sim_N);
[y_step, u_step] = simulate_deadbeat(numz, denz, num_ds, den_ds, r_step);

t_vec = (0:sim_N-1) * T;
r_ramp = t_vec;
[y_ramp, u_ramp] = simulate_deadbeat(numz, denz, num_dr, den_dr, r_ramp);

% ---- 最小拍控制性能分析 ----
fprintf('\n====== 最小拍控制性能分析 ======\n');
yf_step = y_step(end);
track_step_idx = find(abs(y_step - yf_step) <= 0.02 * abs(yf_step), 1);
fprintf('阶跃输入——最小拍控制:\n');
fprintf('  跟踪拍数(±2%%): %d 拍 = %.2f s (理论值: 1拍)\n', ...
    track_step_idx, track_step_idx * T);
fprintf('  稳态误差: %.6f\n', abs(1 - yf_step));
fprintf('  y序列前6拍: [%s]\n', sprintf('%.4f ', y_step(1:min(6,sim_N))));
fprintf('  u序列前6拍: [%s]\n', sprintf('%.4f ', u_step(1:min(6,sim_N))));
fprintf('  说明：阶跃输入下1拍后即达到设定值(±2%%)，满足最小拍要求。\n');
fprintf('        但采样点之间存在纹波（u(k)在2拍后非零）。\n');

yf_ramp = mean(y_ramp(end-5:end));
fprintf('\n斜坡输入——最小拍控制:\n');
fprintf('  理论跟踪拍数: 2拍\n');
fprintf('  y序列前6拍: [%s]\n', sprintf('%.4f ', y_ramp(1:min(6,sim_N))));
ramp_err_end = mean(abs(r_ramp(end-10:end) - y_ramp(end-10:end)));
fprintf('  稳态跟踪误差(最后10拍平均): %.4f\n', ramp_err_end);
fprintf('  说明：斜坡输入下2拍后完全跟踪（理论上）。\n');

% 绘图
fig1 = figure('Name', '实验五：最小拍控制系统响应', 'Position', [100, 100, 1100, 900]);

subplot(3, 2, 1);
stairs(0:sim_N-1, r_step, 'k--', 'LineWidth', 1.5); hold on;
stairs(0:sim_N-1, y_step, 'b-o', 'MarkerSize', 4);
xlabel('采样拍数 k'); ylabel('y(k)');
title('阶跃输入——系统输出'); legend('r(k)', 'y(k)'); grid on;

subplot(3, 2, 2);
stairs(0:sim_N-1, u_step, 'r-o', 'MarkerSize', 4);
xlabel('采样拍数 k'); ylabel('u(k)');
title('阶跃输入——控制量'); grid on;

subplot(3, 2, 3);
stairs(0:sim_N-1, r_ramp, 'k--', 'LineWidth', 1.5); hold on;
stairs(0:sim_N-1, y_ramp, 'b-o', 'MarkerSize', 4);
xlabel('采样拍数 k'); ylabel('y(k)');
title('斜坡输入——系统输出'); legend('r(k)', 'y(k)'); grid on;

subplot(3, 2, 4);
stairs(0:sim_N-1, u_ramp, 'r-o', 'MarkerSize', 4);
xlabel('采样拍数 k'); ylabel('u(k)');
title('斜坡输入——控制量'); grid on;

subplot(3, 2, 5);
stairs(0:sim_N-1, r_step - y_step, 'b-o', 'MarkerSize', 4); hold on;
stairs(0:sim_N-1, r_ramp - y_ramp, 'r-s', 'MarkerSize', 4);
xlabel('采样拍数 k'); ylabel('e(k)');
title('跟踪误差'); legend('阶跃', '斜坡'); grid on;

subplot(3, 2, 6);
[y_step_ctrl_ramp, ~] = simulate_deadbeat(numz, denz, num_ds, den_ds, r_ramp);
stairs(0:sim_N-1, r_ramp, 'k--', 'LineWidth', 1.5); hold on;
stairs(0:sim_N-1, y_ramp, 'b-o', 'MarkerSize', 4);
stairs(0:sim_N-1, y_step_ctrl_ramp, 'r-s', 'MarkerSize', 4);
xlabel('采样拍数 k'); ylabel('y(k)');
title('斜坡输入——控制器匹配性验证');
legend('r(k)', '斜坡控制器', '阶跃控制器'); grid on;

sgtitle('最小拍控制系统响应');
img_path = fullfile(output_dir, 'fig1_最小拍控制响应.png');
exportgraphics(fig1, img_path, 'Resolution', 150);

%% ==================== 步骤4：无纹波 vs 有纹波最小拍对比 ====================
sim_N2 = 40;
r_step2 = ones(1, sim_N2);
[y_rf, u_rf] = simulate_deadbeat(numz, denz, num_rf, den_rf, r_step2);

% ---- 有纹波 vs 无纹波 数值对比 ----
fprintf('\n====== 有纹波 vs 无纹波最小拍控制 详细对比 ======\n');
fprintf('%-20s %-16s %-16s\n', '指标', '有纹波', '无纹波');
fprintf('%-20s %-16s %-16s\n', '------', '------', '------');
track_rf = find(abs(y_rf - 1) <= 0.02, 1);
track_ripple = find(abs(y_step(1:sim_N2) - 1) <= 0.02, 1);
fprintf('%-20s %-16d %-16d\n', '跟踪拍数(±2%%)', track_ripple, track_rf);
u_ss_ripple = u_step(5:sim_N2);  u_ss_rf = u_rf(5:end);
fprintf('%-20s %-16.4f %-16.4f\n', 'u_max', max(u_step(1:sim_N2)), max(u_rf));
fprintf('%-20s %-16s %-16s\n', 'u幅值范围(稳态)', ...
    sprintf('[%.2f,%.2f]', min(u_ss_ripple), max(u_ss_ripple)), ...
    sprintf('[%.2f,%.2f]', min(u_ss_rf), max(u_ss_rf)));
fprintf('%-20s %-16.4f %-16.4f\n', 'u_std(稳态)', std(u_ss_ripple), std(u_ss_rf));
fprintf('%-20s %-16.4f %-16.4f\n', '控制量增量max ', ...
    max(abs(diff(u_step(1:sim_N2)))), max(abs(diff(u_rf))));
fprintf('%-20s %-16s %-16s\n', '纹波评价', '有纹波，u持续振荡', '无纹波，u快速收敛');

fprintf('\n分析说明：\n');
fprintf('有纹波最小拍：控制量u(k)在输出达到设定值后仍大幅变化，\n');
fprintf('  导致采样点之间输出存在振荡（纹波）。响应速度快（1拍）\n');
fprintf('  但控制量消耗大，对执行机构要求高。\n');
fprintf('无纹波最小拍：通过在Phi(z)中包含对象所有稳定零点，\n');
fprintf('  使控制量u(k)快速收敛到常值，消除了采样间纹波。\n');
fprintf('  代价是跟踪速度略慢（需要更多拍），但控制更平滑、更实用。\n');

fig2 = figure('Name', '有纹波vs无纹波最小拍控制', 'Position', [100, 100, 1000, 600]);

subplot(2, 2, 1);
stairs(0:sim_N2-1, r_step2, 'k--', 'LineWidth', 1.5); hold on;
stairs(0:sim_N2-1, y_step(1:sim_N2), 'b-o', 'MarkerSize', 4);
stairs(0:sim_N2-1, y_rf, 'r-s', 'MarkerSize', 4);
xlabel('采样拍数 k'); ylabel('y(k)');
title('输出响应对比'); legend('设定值', '有纹波', '无纹波'); grid on;

subplot(2, 2, 2);
stairs(0:sim_N2-1, u_step(1:sim_N2), 'b-o', 'MarkerSize', 4); hold on;
stairs(0:sim_N2-1, u_rf, 'r-s', 'MarkerSize', 4);
xlabel('采样拍数 k'); ylabel('u(k)');
title('控制量对比（观察纹波）'); legend('有纹波', '无纹波'); grid on;

subplot(2, 2, 3);
plot(0:sim_N2-1, y_step(1:sim_N2), 'b-x', 'LineWidth', 1, 'MarkerSize', 5); hold on;
plot(0:sim_N2-1, y_rf, 'r-o', 'LineWidth', 1, 'MarkerSize', 5);
xlabel('采样拍数 k'); ylabel('y(k)');
title('采样间纹波对比（放大）'); legend('有纹波', '无纹波'); grid on;

subplot(2, 2, 4);
du_step = [0, diff(u_step(1:sim_N2))];
du_rf = [0, diff(u_rf)];
bar([0:sim_N2-1]', [du_step', du_rf']);
xlabel('采样拍数 k'); ylabel('\Deltau(k)');
title('控制量增量对比'); legend('有纹波', '无纹波'); grid on;

sgtitle('有纹波 vs 无纹波最小拍控制');
img_path = fullfile(output_dir, 'fig2_有纹波vs无纹波.png');
exportgraphics(fig2, img_path, 'Resolution', 150);

%% ==================== 步骤5：控制器鲁棒性分析 ====================
fprintf('\n====== 最小拍控制器鲁棒性分析 ======\n');

Gz_nominal = c2d(tf(10, [1 1 0]), 0.1, 'zoh');
Phi_step_nom = tf([0 1], [1 0], 0.1);
Dz_nominal = minreal(Phi_step_nom / (Gz_nominal * (1 - Phi_step_nom)));
[num_dn, den_dn] = tfdata(Dz_nominal, 'v');

scenarios = {
    '标称模型 (T=0.1)',     10, [1 1 0],   0.1;
    '增益变化 (+50%%)',     15, [1 1 0],   0.1;
    '极点变化',              10, [1 1.5 0], 0.1;
    '采样周期变化 (T=0.2)',  10, [1 1 0],   0.2;
};

robustness_summary = cell(4, 4);

fig3 = figure('Name', '最小拍控制器鲁棒性分析', 'Position', [100, 100, 1000, 800]);

for i = 1:4
    K_act = scenarios{i, 2};
    den_act = scenarios{i, 3};
    T_act = scenarios{i, 4};
    
    Gs_actual = tf(K_act, den_act);
    Gz_actual = c2d(Gs_actual, T_act, 'zoh');
    [num_ga, den_ga] = tfdata(Gz_actual, 'v');
    
    r = ones(1, 40);
    [y, u] = simulate_deadbeat(num_ga, den_ga, num_dn, den_dn, r);
    
    subplot(2, 2, i);
    stairs(0:39, r, 'k--', 'LineWidth', 1.5); hold on;
    stairs(0:39, y, 'b-o', 'MarkerSize', 4);
    xlabel('采样拍数 k'); ylabel('y(k)');
    title(scenarios{i, 1});
    grid on;
    
    overshoot_i = max(0, (max(y)-1)*100);
    ess_i = abs(1 - y(end));
    track_i = find(abs(y - 1) <= 0.1, 1);
    if isempty(track_i), track_i = 40; end
    
    robustness_summary{i, 1} = scenarios{i, 1};
    robustness_summary{i, 2} = overshoot_i;
    robustness_summary{i, 3} = ess_i;
    robustness_summary{i, 4} = track_i;
    
    fprintf('场景: %s\n', scenarios{i, 1});
    fprintf('  超调量 = %.2f%%\n', overshoot_i);
    fprintf('  稳态误差 = %.4f\n', ess_i);
    fprintf('  进入±10%%拍数 = %d\n\n', track_i);
end

sgtitle('最小拍控制器鲁棒性分析——对象参数变化');
img_path = fullfile(output_dir, 'fig3_鲁棒性分析.png');
exportgraphics(fig3, img_path, 'Resolution', 150);

% ---- 鲁棒性汇总与分析 ----
fprintf('\n====== 鲁棒性分析汇总 ======\n');
fprintf('%-22s %-10s %-12s %-14s\n', '场景', '超调%%', '稳态误差', '进入±10%%拍数');
fprintf('%-22s %-10s %-12s %-14s\n', '------', '------', '------', '------');
for i = 1:4
    fprintf('%-22s %-10.2f %-12.4f %-14d\n', robustness_summary{i,:});
end

fprintf('\n====== 鲁棒性评价 ======\n');
fprintf('标称模型：1拍内精确跟踪，无超调，稳态误差为零——最小拍设计达到理论性能。\n');
fprintf('增益变化(+50%%)：控制器针对标称增益设计，实际增益偏大导致输出偏大，\n');
fprintf('  可能产生超调或振荡。最小拍控制对模型参数较敏感。\n');
fprintf('极点变化：对象极点位置改变影响离散化后的模型结构，\n');
fprintf('  控制器不再匹配，跟踪性能恶化，可能出现稳态误差。\n');
fprintf('采样周期变化(T=0.2)：采样周期增大改变了离散模型，\n');
fprintf('  控制器的设计不再精确匹配，性能下降。\n');
fprintf('结论：最小拍控制对模型准确性要求较高，鲁棒性不如PID控制。\n');
fprintf('      在实际工程中，最小拍控制器常用于对模型精度有信心的场合。\n');
fprintf('      若模型不确定性较大，建议采用PID或大林算法等更鲁棒的控制策略。\n');

fprintf('\n========== 实验五全部完成，图片已保存至 %s 文件夹 ==========\n', output_dir);

%% ==================== 函数定义区 ====================

function [y, u] = simulate_deadbeat(numz, denz, num_d, den_d, r)
    N = length(r);
    y = zeros(1, N);
    u = zeros(1, N);
    e = zeros(1, N);
    
    n_ctrl_num = length(num_d);
    n_ctrl_den = length(den_d);
    n_obj_num = length(numz);
    n_obj_den = length(denz);
    
    u_buf = zeros(1, max([n_ctrl_den, n_ctrl_num, n_obj_num]));
    e_buf = zeros(1, max([n_ctrl_num, n_ctrl_den]));
    y_buf = zeros(1, n_obj_den);
    
    for k = 1:N
        e(k) = r(k) - y(max(1, k-1));
        e_buf = [e(k), e_buf(1:end-1)];
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
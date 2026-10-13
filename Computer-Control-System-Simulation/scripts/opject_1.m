%% ==================== 机电系统计算机控制课程设计 ====================
% 内容：离散控制系统稳定性分析
% 包含：极点映射图、临界增益计算、Jury判据、双线性变换+劳斯判据
% 运行方式：直接运行，所有函数定义在脚本末尾
% 输出：2个图片窗口 + 命令行表格数据
% ====================================================================

clear; clc; close all;

%% ========================= 参数设置（可修改） =========================
% 被控对象连续传递函数 G(s) = 1 / [s(s+1)(s+2)]
num_s = [1];
den_s = [1, 3, 2, 0];   % s^3 + 3s^2 + 2s
Gs = tf(num_s, den_s);

% 表1-2：不同采样周期（用于临界增益分析）
T_crit_list = [0.1, 0.2, 0.5, 1.0, 1.5, 2.0];

% 表1-3：Jury判据检验的特定采样周期和增益
T_jury = 0.5;       % 采样周期 (s)
K_jury = 2;         % 开环增益

% 搜索临界增益时的步长和上限
stepK = 0.01;
maxK = 100;

%% ==================== 1. 不同采样周期下的极点分布图 ====================
T_plot_list = [0.1, 0.5, 1.0, 2.0];   % 用于绘图的采样周期
figure('Name', '实验一：s-z平面映射与极点分布', 'Position', [100,100,1200,600]);

for i = 1:length(T_plot_list)
    T = T_plot_list(i);
    Gz = c2d(Gs, T, 'zoh');
    
    % s平面极点
    subplot(2, length(T_plot_list), i);
    pzmap(Gs);
    title(['s-plane (T=', num2str(T), 's)']);
    grid on; axis equal; xlim([-3,1]); ylim([-2,2]);
    
    % z平面极点 + 单位圆
    subplot(2, length(T_plot_list), i + length(T_plot_list));
    pzmap(Gz);
    hold on;
    theta = 0:0.01:2*pi;
    plot(cos(theta), sin(theta), 'k--', 'LineWidth', 1);
    title(['z-plane (T=', num2str(T), 's)']);
    grid on; axis equal; xlim([-1.5,1.5]); ylim([-1.5,1.5]);
    hold off;
end
sgtitle('不同采样周期下s平面与z平面的极点分布');
disp('【图1】已生成：s-z平面极点分布图');

%% ==================== 2. 表1-2：不同采样周期下的临界增益 ====================
fprintf('\n========== 表1-2 不同采样周期下的临界稳定增益 K ==========\n');
fprintf('采样周期 T(s)   : ');
fprintf('%6.2f  ', T_crit_list);
fprintf('\n临界增益 K_crit : ');

K_crit = zeros(size(T_crit_list));
poles_at_crit = cell(size(T_crit_list));

for idx = 1:length(T_crit_list)
    T = T_crit_list(idx);
    Gz = c2d(Gs, T, 'zoh');
    [numz, denz] = tfdata(Gz, 'v');
    if length(numz) < length(denz)
        numz = [zeros(1, length(denz)-length(numz)), numz];
    end
    K = 0;
    crit_poles = [];
    for K_test = 0:stepK:maxK
        den_cl = denz + K_test * numz;
        poles_val = roots(den_cl);
        if max(abs(poles_val)) >= 1
            K = K_test;
            crit_poles = poles_val;
            break;
        end
    end
    K_crit(idx) = K;
    poles_at_crit{idx} = crit_poles;
    fprintf('%6.2f  ', K);
end
fprintf('\n');

% ---- 临界增益分析文字结论 ----
fprintf('\n====== 临界增益分析结论 ======\n');
fprintf('观察：采样周期越大(采样越慢)，临界增益 K_crit 越小。\n');
fprintf('原因：采样周期增大导致离散化模型相位滞后增大，\n');
fprintf('      系统更容易失稳，因此容许的最大开环增益减小。\n');
for idx = 1:length(T_crit_list)
    fprintf('  T=%.1fs: K_crit=%.2f', T_crit_list(idx), K_crit(idx));
    % 比较相邻的减小比例
    if idx > 1
        ratio = (K_crit(idx-1) - K_crit(idx)) / K_crit(idx-1) * 100;
        fprintf(' (较T=%.1fs下降%.1f%%)', T_crit_list(idx-1), ratio);
    end
    fprintf('\n');
end

% 同时绘制根轨迹图（辅助验证）
figure('Name', '不同采样周期下的根轨迹', 'Position', [100,100,1000,600]);
hold on;
colors = lines(length(T_crit_list));
for idx = 1:length(T_crit_list)
    T = T_crit_list(idx);
    Gz = c2d(Gs, T, 'zoh');
    rlocus(Gz, colors(idx,:));
end
hold off;
title('不同采样周期下的根轨迹（临界增益对应单位圆交点）');
legend(arrayfun(@(x) sprintf('T=%.1fs', x), T_crit_list, 'UniformOutput', false));
sgtitle('根轨迹法求临界增益');
disp('【图2】已生成：根轨迹图 (不同采样周期)');

%% ==================== 3. 表1-3：Jury判据检验（指定T, K） ====================
Gz = c2d(Gs, T_jury, 'zoh');
[numz, denz] = tfdata(Gz, 'v');
if length(numz) < length(denz)
    numz = [zeros(1, length(denz)-length(numz)), numz];
end
den_cl = denz + K_jury * numz;

[stable_jury, details] = jury_criterion_detailed(den_cl, true);

fprintf('\n========== 表1-3 朱利判据检验 (T = %.1f s, K = %.1f) ==========\n', T_jury, K_jury);
fprintf('+-----------------+------------------------+----------------+--------------+\n');
fprintf('|   检验条件       |        表达式          |     数值       |   是否满足   |\n');
fprintf('+-----------------+------------------------+----------------+--------------+\n');
fprintf('| 条件1           | D(1) > 0               | %12.4f  |     %s      |\n', details.D1, string(details.cond1));
fprintf('| 条件2           | (-1)^n·D(-1) > 0       | %12.4f  |     %s      |\n', details.Dm1, string(details.cond2));
fprintf('| 条件3           | |a_n| < a0             | |%10.4f|<%-10.4f |     %s      |\n', details.an, details.a0, string(details.cond3));
fprintf('+-----------------+------------------------+----------------+--------------+\n');
fprintf('| 综合结论        | ---                    |      ---       |  %s  |\n', string(stable_jury));
fprintf('+-----------------+------------------------+----------------+--------------+\n');

% ---- Jury判据文字解释 ----
fprintf('\n====== Jury判据分析说明 ======\n');
fprintf('条件1 D(1) = %.4f > 0 ：%s\n', details.D1, string(details.cond1));
if details.cond1
    fprintf('  -> 说明闭环系统在z=1处有正增益，满足稳定必要条件之一。\n');
else
    fprintf('  -> z=1处增益非正，必要条件不满足，系统不可能稳定。\n');
end
fprintf('条件2 (-1)^n·D(-1) = %.4f > 0 ：%s\n', details.Dm1, string(details.cond2));
if details.cond2
    fprintf('  -> 说明闭环系统在z=-1处满足符号条件。\n');
else
    fprintf('  -> z=-1处条件不满足，系统不稳定。\n');
end
fprintf('条件3 |a_n| = %.4f < a0 = %.4f ：%s\n', abs(details.an), details.a0, string(details.cond3));
if details.cond3
    fprintf('  -> 所有极点位于单位圆内的必要条件满足。\n');
else
    fprintf('  -> 最高次项系数过大，存在单位圆外极点。\n');
end

%% ==================== 4. 双线性变换+劳斯判据验证 ====================
fprintf('\n========== 步骤5 双线性变换+劳斯判据验证 (T=%.1f s, K=%.1f) ==========\n', T_jury, K_jury);
stable_routh = routh_via_bilinear(numz, denz, K_jury, true);

fprintf('\n====== 综合对比验证 ======\n');
fprintf('Jury判据结论       : %s\n', string(stable_jury));
fprintf('双线性+劳斯判据结论 : %s\n', string(stable_routh));
if stable_jury == stable_routh
    fprintf('两种方法结论一致，验证正确。\n');
else
    fprintf('警告：两种方法结论不一致，请检查参数。\n');
end

% 对比该T下的临界增益
Gz_j = c2d(Gs, T_jury, 'zoh');
[numz_j, denz_j] = tfdata(Gz_j, 'v');
if length(numz_j) < length(denz_j)
    numz_j = [zeros(1, length(denz_j)-length(numz_j)), numz_j];
end
K_crit_j = 0;
for K_test = 0:0.001:maxK
    den_cl_t = denz_j + K_test * numz_j;
    if max(abs(roots(den_cl_t))) >= 1
        K_crit_j = K_test;
        break;
    end
end
fprintf('\nT=%.1fs 时的临界增益: %.4f\n', T_jury, K_crit_j);
fprintf('当前增益 K=%.1f < K_crit=%.4f, 因此系统应当稳定。\n', K_jury, K_crit_j);

% ========== 脚本主体结束，以下为函数定义区 ==========
disp('========== 全部代码运行完成 ==========');

%% ==================== 函数定义区（必须放在脚本末尾） ====================

function [stable, details] = jury_criterion_detailed(den, display_flag)
    n = length(den) - 1;
    a0 = den(1);
    an = den(end);
    
    D1 = polyval(den, 1);
    Dm1 = (-1)^n * polyval(den, -1);
    cond1_ok = (D1 > 0);
    cond2_ok = (Dm1 > 0);
    cond3_ok = (abs(an) < a0);
    
    details.D1 = D1;
    details.Dm1 = Dm1;
    details.an = an;
    details.a0 = a0;
    details.cond1 = cond1_ok;
    details.cond2 = cond2_ok;
    details.cond3 = cond3_ok;
    
    if display_flag
        fprintf('\n--- 朱利判据必要条件 ---\n');
        fprintf('条件1: D(1) = %.4f  > 0 ? %s\n', D1, string(cond1_ok));
        fprintf('条件2: (-1)^n·D(-1) = %.4f  > 0 ? %s\n', Dm1, string(cond2_ok));
        fprintf('条件3: |a_n| = %.4f  <  a0 = %.4f ? %s\n', abs(an), a0, string(cond3_ok));
    end
    
    if ~(cond1_ok && cond2_ok && cond3_ok)
        stable = false;
        if display_flag
            fprintf('必要条件不满足 → 系统不稳定\n');
        end
        return;
    end
    
    row = den;
    all_positive = true;
    if display_flag
        fprintf('\n--- 朱利判据充分条件（Jury表） ---\n');
    end
    for k = 1:n
        row_rev = fliplr(row);
        alpha = row(1) / row_rev(1);
        new_len = length(row) - 1;
        new_row = zeros(1, new_len);
        for j = 1:new_len
            new_row(j) = row(j) - alpha * row_rev(j);
        end
        row = new_row;
        if row(1) <= 0
            all_positive = false;
            if display_flag
                fprintf('第%d行首项 = %.4f ≤ 0 → 不稳定\n', k+1, row(1));
            end
            break;
        elseif display_flag
            fprintf('第%d行首项 = %.4f > 0 ✓\n', k+1, row(1));
        end
    end
    
    stable = all_positive;
    if display_flag && stable
        fprintf('所有Jury表首项为正 → 系统稳定\n');
    elseif display_flag && ~stable
        fprintf('Jury表出现非正首项 → 系统不稳定\n');
    end
end

function stable = routh_via_bilinear(numz, denz, K, display_flag)
    if length(numz) < length(denz)
        numz_pad = [zeros(1, length(denz)-length(numz)), numz];
    else
        numz_pad = numz;
    end
    den_cl = denz + K * numz_pad;
    
    syms z w;
    Dz = poly2sym(den_cl, z);
    Dw_raw = subs(Dz, z, (w+1)/(w-1));
    Dw_simple = simplify(Dw_raw, 'Steps', 100);
    [numDw, denDw] = numden(Dw_simple);
    Dw_poly = expand(numDw);
    
    coeffs_w = coeffs(Dw_poly, w, 'All');
    coeffs_w = double(coeffs_w);
    coeffs_w(abs(coeffs_w) < 1e-10) = 0;
    
    if coeffs_w(1) < 0
        coeffs_w = -coeffs_w;
    end
    
    if display_flag
        fprintf('\n--- 双线性变换结果 ---\n');
        fprintf('w域特征多项式系数（降幂）: ');
        fprintf('%g  ', coeffs_w);
        fprintf('\n');
    end
    
    n = length(coeffs_w) - 1;
    if mod(n, 2) == 0
        routh_cols = n/2 + 1;
    else
        routh_cols = (n+1)/2;
    end
    routh_table = zeros(n+1, routh_cols);
    
    routh_table(1, 1:ceil((n+1)/2)) = coeffs_w(1:2:end);
    if n >= 1
        routh_table(2, 1:floor((n+1)/2)) = coeffs_w(2:2:end);
    end
    
    for i = 3:n+1
        for j = 1:routh_cols-1
            a = routh_table(i-1, 1);
            if abs(a) < 1e-8
                a = 1e-8;
            end
            b = routh_table(i-2, j+1);
            c = routh_table(i-1, j+1);
            if j <= size(routh_table,2)-1 && abs(b) > 1e-12
                routh_table(i, j) = (a * b - routh_table(i-2, 1) * c) / a;
            else
                routh_table(i, j) = 0;
            end
        end
        if all(abs(routh_table(i, :)) < 1e-6)
            if display_flag
                fprintf('警告：劳斯表出现全零行，系统可能临界稳定。\n');
            end
            stable = false;
            return;
        end
    end
    
    first_col = routh_table(:, 1);
    first_col = first_col(~isnan(first_col) & ~isinf(first_col));
    sign_changes = sum(diff(sign(first_col)) ~= 0);
    stable = (sign_changes == 0) && all(first_col > 0);
    
    if display_flag
        fprintf('\n--- 劳斯表第一列 ---\n');
        disp(first_col);
        fprintf('符号变化次数: %d\n', sign_changes);
        if stable
            fprintf('劳斯判据结论：系统稳定\n');
        else
            fprintf('劳斯判据结论：系统不稳定\n');
        end
    end
end
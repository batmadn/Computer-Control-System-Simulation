# MATLAB 控制系统实验结果（完整图片汇总）

> 说明：本页为 MATLAB 控制系统课程实验截图，**不代表 RoboCup/GNN 比赛效果**。图片由原始实验截图整理；部分仿真存在明显振荡、发散或异常指标，保留作为实验现象，不应视为算法改进成功的证明。

## 实验 1：离散系统稳定性分析

### Jury 判据命令行输出
![Jury 判据命令行输出](images/control/experiment01/jury_test_output.png)

### 不同采样周期下的根轨迹
![不同采样周期下的根轨迹](images/control/experiment01/sampling_root_locus.png)

### s-z 平面极点分布
![s-z 平面极点分布](images/control/experiment01/s_z_pole_mapping.png)

### Jury 判据条件表
![Jury 判据条件表](images/control/experiment01/jury_test_table.png)

### 不同采样周期的临界增益
![不同采样周期的临界增益](images/control/experiment01/critical_gain_table.png)

### 根轨迹法求临界增益
![根轨迹法求临界增益](images/control/experiment01/critical_gain_root_locus.png)

## 实验 2：离散系统时域与频域分析

### 离散系统频域特性分析
![离散系统频域特性分析](images/control/experiment02/frequency_analysis.png)

### 采样周期对幅频特性的影响
![采样周期对幅频特性的影响](images/control/experiment02/sampling_frequency_response.png)

### 阶跃响应指标与稳态误差输出
![阶跃响应指标与稳态误差输出](images/control/experiment02/time_response_metrics.png)

### 不同采样周期的单位阶跃响应
![不同采样周期的单位阶跃响应](images/control/experiment02/step_responses.png)

## 实验 3：数字 PID 控制器设计

### Z-N 参数整定结果
![Z-N 参数整定结果](images/control/experiment03/zn_parameter_table.png)

### 临界振荡验证
![临界振荡验证](images/control/experiment03/critical_oscillation.png)

### 不同 PID 参数下的阶跃响应对比
![不同 PID 参数下的阶跃响应对比](images/control/experiment03/pid_tuning_comparison.png)

### PID 离散化方法频率响应对比
![PID 离散化方法频率响应对比](images/control/experiment03/pid_discretization_comparison.png)

### 比例增益 Kp 对系统响应的影响
![比例增益 Kp 对系统响应的影响](images/control/experiment03/kp_sensitivity.png)

### 不同 PID 参数的性能指标
![不同 PID 参数的性能指标](images/control/experiment03/pid_metrics.png)

### Z-N 临界参数
![Z-N 临界参数](images/control/experiment03/zn_critical_parameters.png)

## 实验 4：Smith 预估控制

### Smith 预估器与普通 PI 控制对比
![Smith 预估器与普通 PI 控制对比](images/control/experiment04/smith_vs_pi.png)

### Smith 预估器模型失配响应
![Smith 预估器模型失配响应](images/control/experiment04/model_mismatch.png)

### Smith 预估器 λ 参数影响
![Smith 预估器 λ 参数影响](images/control/experiment04/lambda_sensitivity.png)

### Smith λ 参数性能指标
![Smith λ 参数性能指标](images/control/experiment04/lambda_metrics.png)

### Smith 与 PI 性能指标对比
![Smith 与 PI 性能指标对比](images/control/experiment04/smith_pi_metrics.png)

### Smith 模型失配指标
![Smith 模型失配指标](images/control/experiment04/model_mismatch_metrics.png)


## 实验 5：最小拍控制

### 有纹波与无纹波最小拍控制对比
![有纹波与无纹波最小拍控制对比](images/control/experiment05/ripple_comparison.png)

### 最小拍控制鲁棒性分析
![最小拍控制鲁棒性分析](images/control/experiment05/robustness_analysis.png)

### 最小拍鲁棒性性能指标
![最小拍鲁棒性性能指标](images/control/experiment05/robustness_metrics.png)

### 最小拍控制阶跃及斜坡响应
![最小拍控制阶跃及斜坡响应](images/control/experiment05/deadbeat_response.png)

### 最小拍性能指标
![最小拍性能指标](images/control/experiment05/deadbeat_metrics.png)


## 实验 6：Dahlin（大林）控制

### 大林控制性能指标
![大林控制性能指标](images/control/experiment06/dahlin_metrics.png)

### 大林控制振铃极点分析
![大林控制振铃极点分析](images/control/experiment06/ringing_poles.png)

### 振铃修正因子输出
![振铃修正因子输出](images/control/experiment06/ringing_correction.png)

### 大林算法控制器设计参数
![大林算法控制器设计参数](images/control/experiment06/controller_parameters.png)

### 大林算法 λ 参数影响
![大林算法 λ 参数影响](images/control/experiment06/dahlin_lambda_sensitivity.png)

### 大林算法振铃消除前后对比
![大林算法振铃消除前后对比](images/control/experiment06/ringing_before_after.png)

### 大林算法与最小拍控制对比
![大林算法与最小拍控制对比](images/control/experiment06/dahlin_vs_deadbeat.png)

### 大林控制器 λ 设计参数
![大林控制器 λ 设计参数](images/control/experiment06/dahlin_lambda_design.png)

### 可能产生振铃的极点
![可能产生振铃的极点](images/control/experiment06/ringing_risk_poles.png)


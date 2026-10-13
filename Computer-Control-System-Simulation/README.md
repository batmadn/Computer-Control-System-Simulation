# Computer Control System Simulation

基于 MATLAB 的计算机控制系统设计与仿真课程项目，包含 6 个相互独立的实验脚本，涵盖离散系统分析、数字 PID、Smith 预估控制、最小拍控制和 Dahlin 控制。

> 项目性质：本科课程设计与控制理论仿真练习。并非真实机器人导航系统或硬件控制部署项目。

## 实验内容

| 脚本 | 实验 | 主要内容 |
| --- | --- | --- |
| `scripts/opject_1.m` | 离散系统稳定性分析 | 极点映射、临界增益、Jury 判据、双线性变换与劳斯判据 |
| `scripts/opject_2.m` | 离散系统性能分析 | 时域响应、稳态误差、频域特性 |
| `scripts/opject_3.m` | 数字 PID 控制 | Ziegler–Nichols 整定、参数与离散化方法对比 |
| `scripts/opject_4.m` | Smith 预估控制 | 纯滞后系统、Smith 与 PI 对比、模型失配分析 |
| `scripts/opject_5.m` | 最小拍控制 | 阶跃/斜坡输入、无纹波设计、鲁棒性分析 |
| `scripts/opject_6.m` | Dahlin 控制 | lambda 参数、振铃现象及控制器比较 |

## 运行环境

- MATLAB R2023b（原项目使用环境）
- Control System Toolbox（使用 `tf`、`c2d`、`feedback` 等函数）

## 如何运行

1. 用 MATLAB 打开 `scripts/` 目录。
2. 将 MATLAB **当前工作目录**设置为 `scripts/`。
3. 单独运行 `opject_1.m` 至 `opject_6.m` 中需要的脚本。
4. 脚本 2–6 会在当前工作目录下创建相应的 `opject_N_images/` 文件夹并导出 PNG 图像；脚本 1 主要显示绘图窗口并输出命令行数据。

## 目录结构

```text
Computer-Control-System-Simulation/
├── README.md
├── .gitignore
└── scripts/
    ├── opject_1.m
    ├── opject_2.m
    ├── opject_3.m
    ├── opject_4.m
    ├── opject_5.m
    └── opject_6.m
```

## 说明

仓库保留原始脚本及其原始文件名 `opject_*.m`，未对算法结果进行重新验证，也未声称进行了真实机器人实验。可在 MATLAB 中复现实验并补充生成的结果图。

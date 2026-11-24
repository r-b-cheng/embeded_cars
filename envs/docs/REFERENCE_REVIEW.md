# 参考项目核查与实现选择

核查日期：2026-11-24。GitHub star 数取自本次 GitHub REST 返回，后续会变化。

| 项目 | 当次 star 数 | 本次用途 |
|---|---:|---|
| hanruihua/ir-sim | 1,245 | 查阅 README：配置驱动的二维仿真、step/render/reset 分离、碰撞检测、轻量可视化 |
| AtsushiSakai/PythonRobotics | 30,964 | 核查仓库元信息，作为后续机器人算法示例索引；本次没有移植它的算法代码 |

IR-SIM 满足用户要求的至少 150 star。README 说明其采用 YAML 场景描述、Python 运行循环和 Matplotlib 视图。本项目借鉴这些接口组织方式，针对单超声波、IMU、编码器和严格独立验收，实现一个单车专用内核。

**没有声称“没找到仓库”。也没有将参考项目代码复制进来。** 当前实现不依赖 IR-SIM，不是其分支或兼容层；现有地图格式也不承诺能直接导入 IR-SIM。

选择自有轻量内核的原因：实验重点是明确的传感器模型、稳定 API、采样时序和可对照的数学精度。避免为此引入多智能体、行为策略和完整接触动力学依赖。现有成熟模拟器的刚体/复杂场景能力比本版本广，本版本聚焦课程的平面差速车。

核查来源：

- https://api.github.com/repos/hanruihua/ir-sim
- https://github.com/hanruihua/ir-sim/blob/main/README.md
- https://ir-sim.readthedocs.io/en/stable/
- https://api.github.com/repos/AtsushiSakai/PythonRobotics
- https://github.com/AtsushiSakai/PythonRobotics

IR-SIM 元信息为 MIT；PythonRobotics 的本次元信息没有返回标准 SPDX 许可标识，因此这里不替它断言整个仓库的许可。
本次未复制这两个项目的源代码。NumPy/Matplotlib/Shapely 等依赖分别适用各自许可证。

## 目标仓库集成状态

通过已连接 GitHub 工具核查的目标：`r-b-cheng/embeded_cars`，默认分支 `main`。
核查到的树/提交标识为 `15dba189ae1120cc9ed46f311406d73dcfc10671`。

本交付仅新增 `envs/`，没有改动 `patrol/`、`wheel_drive/`、舵机程序或其他既有文件。
当前连接提供读取操作；会话内未暴露提交/推送操作，本地网络也不能访问 GitHub。
所以交付附带本地 Git 提交的补丁，**远端尚未提交**。不能将本地提交号当成远端已存在的 commit。

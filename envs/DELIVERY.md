# Python 小车仿真环境 · GitHub 交付说明

## 远端状态

本目录已通过 GitHub 插件上传至仓库 `r-b-cheng/embeded_cars` 的分支 `add-python-envs-20261124`，包含 74 个文件（源码、10 张地图、测试脚本和测试产物）。合并到 `main` 前请查看该分支或对应 Pull Request。

## 本地运行

```bash
git clone https://github.com/r-b-cheng/embeded_cars.git
cd embeded_cars
git switch add-python-envs-20261124
cd envs
python -m pip install -e ".[visual,test]"
python -m pytest -q
python -m robot_sim.demo --map maps/01_empty_room.json --pose 1 1 0 --interactive
```

## 验证证据

- `reports/TEST_REPORT.md`：测试范围、精度和限制。
- `reports/acceptance.json`：10 张地图 × 10 个起点 × 20 段运动的验收统计。
- `reports/acceptance_trace.jsonl.gz`：传感器及轨迹验收记录。
- `reports/gallery.html`：地图和起始位置预览。
- `reports/source_sha256.json`：被测源码、配置及地图的 SHA-256 摘要。

GitHub 插件上传时逐个核验了全部 74 个原始文件的 Git blob SHA；本文件仅更新了交付状态说明。算法与传感器模型的正确性仍以 `tests/` 和实机验证为准。

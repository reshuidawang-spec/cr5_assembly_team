# 标定数据索引

机器可读的唯一入口是 `config/calibration_registry.json`。脚本不得通过文件名中的
`x_neg`、`y_neg` 或 `needle3` 猜测物理测针身份。

## 当前有效基准

| 对象 | ID | 规范文件 | 状态 |
|---|---|---|---|
| 20 mm 标准球 | `standard_sphere_20mm_setup_202606` | `data/2026.6.16/yneg_near_fourth_refit_20260616.json` | accepted |
| 当前实体 y_neg 侧针 | `stylus_cross_y_neg_20260617` | `data/2026.6.21/y_neg_auto_calibrated_fit_20260621.json` | auto_calibrated |

标准球球心（`base_link`，mm）：

```text
[-401.8971761006165, 126.48848592818706, 89.57111951578374]
```

当前 y_neg 实体针球心相对法兰偏置（mm）：

```text
[-30.65070976670566, -22.24271298559857, 174.8500025813552]
```

注意：标准球规范文件中的 `local_ball_offset_mm` 属于另一颗完成 10 点绝对标定的
参考侧针，不能因为历史标签同为 `y_neg` 就用于当前实体针。

## 尚不可自动运行

| 对象 | 注册表 ID | 原因 |
|---|---|---|
| 10 点参考侧针 | `stylus_reference_full_01` | 拟合质量好，但尚未重新确认其当前物理针号 |
| 用户编号针3 | `stylus_needle3_x_neg_20260617` | 只有两个近同姿态触点，三维 offset 约束不足 |
| x_pos 粗种子 | `stylus_cross_x_pos_rough_20260617` | 人工方向投影，样本离散 9.36 mm |
| y_pos 粗种子 | `stylus_cross_y_pos_rough_20260617` | 人工方向投影，样本离散 9.37 mm |
| x_neg 手动组 | `stylus_cross_x_neg_invalid_20260617` | 三行数据混杂，样本离散 37.76 mm |

这些文件保留用于追溯和重新拟合，不得作为真实运动输入。完整状态和原因以注册表为准。

## 数据目录规则

- `data/2026.6.16/`：可信参考侧针绝对标定及逐点验证，球心来源。
- `data/2026.6.17/`：五针人工示教、针3试验及派生粗种子；多数不是最终标定。
- `data/2026.6.21/`：球心审计、当前 y_neg 固定球心拟合及自动流程故障记录。
- `data/archive_obsolete_20260603/`：明确废弃的旧试验，不作为任何默认输入。
- 根目录 `data/*.json/csv`：历史试验。只有注册表明确引用的文件才可作为规范输入。

不要移动或覆盖原始 CSV。新采集使用新的日期目录和唯一 `session_id`；新拟合输出新文件，
通过更新注册表切换规范版本。旧文件应在注册表 `deprecated_artifacts` 中标记状态和替代文件。

## 使用前校验

```bash
./scripts/calibration_registry.py

./scripts/calibration_registry.py \
  --reference-fit-json data/2026.6.16/yneg_near_fourth_refit_20260616.json \
  --branches y_neg \
  --branch-fit y_neg=data/2026.6.21/y_neg_auto_calibrated_fit_20260621.json \
  --branch-stylus y_neg=stylus_cross_y_neg_20260617 \
  --require-auto-ready
```

真实自动测量还会在启动 ROS/MoveIt 和下发运动前执行同一校验。

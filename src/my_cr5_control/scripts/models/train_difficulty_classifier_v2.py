#!/usr/bin/env python3
"""
场景难度分类器训练脚本 V2 (Scene Difficulty Classifier Training V2)

改进：使用simple_random_training_table.csv中的完整几何特征
输入：场景几何特征（clearance, depth_ratio, lateral_offset等）
输出：三分类模型 (easy/medium/hard)

难度定义：基于RRTConnect的成功率和规划时间
- easy: 成功率>=90% 且 平均时间<1000ms
- medium: 成功率>=50% 或 平均时间<5000ms
- hard: 成功率<50% 且 平均时间>=5000ms
"""

import argparse
import csv
import json
from collections import defaultdict
from datetime import datetime
from pathlib import Path

import numpy as np
import xgboost as xgb
from sklearn.metrics import classification_report, confusion_matrix
from sklearn.model_selection import train_test_split

# 项目根目录
REPO_ROOT = Path(__file__).resolve().parents[2]
TRAINING_TABLE_PATH = REPO_ROOT / "test_results" / "exports" / "simple_random_training_table.csv"
OUTPUT_DIR = REPO_ROOT / "test_results" / "models" / "difficulty_classifier"

# 选择的几何特征（对应feature_profiles.csv中的geometric profile）
GEOMETRIC_FEATURES = [
    "difficulty_score",           # 场景难度评分
    "estimated_clearance_m",      # 估计间隙
    "clearance_margin_m",         # 间隙余量
    "depth_ratio",                # 深度比
    "radial_offset_m",            # 径向偏移
    "lateral_offset_norm_m",      # 横向偏移范数
    "approach_distance_m",        # 接近距离
    "clearance_to_opening_ratio", # 间隙开口比
    "local_opening_size_m",       # 局部开口尺寸
    "box_width_m",                # 箱体宽度
    "box_depth_m",                # 箱体深度
    "box_height_m",               # 箱体高度
    "hole_radius_m",              # 孔半径
    "hole_depth_m",               # 孔深度
    "top_depth_m",                # 顶部深度
]


def load_training_table(table_path: Path):
    """加载训练表数据"""
    rows = []
    with table_path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    print(f"加载了 {len(rows)} 条记录")
    return rows


def extract_task_difficulty_labels(rows):
    """提取任务级别的难度标签（基于RRTConnect性能）"""
    # 按task_uid分组
    task_data = defaultdict(list)

    for row in rows:
        planner = row.get("planner", "")
        if planner != "RRTConnect":
            continue

        task_uid = row.get("task_uid", "")
        success = int(row.get("success", "0"))
        wall_time_ms = float(row.get("wall_time_ms", "0"))

        task_data[task_uid].append({
            "success": success,
            "wall_time_ms": wall_time_ms,
        })

    # 计算每个任务的难度标签
    task_difficulty = {}
    for task_uid, records in task_data.items():
        if not records:
            continue

        success_count = sum(r["success"] for r in records)
        total = len(records)
        success_rate = success_count / total if total > 0 else 0

        # 计算平均规划时间（仅成功的）
        success_times = [r["wall_time_ms"] for r in records if r["success"]]
        avg_time = np.mean(success_times) if success_times else 10000.0

        # 难度判定逻辑
        if success_rate >= 0.9 and avg_time < 1000:
            difficulty = "easy"
        elif success_rate >= 0.5 or avg_time < 5000:
            difficulty = "medium"
        else:
            difficulty = "hard"

        task_difficulty[task_uid] = difficulty

    print(f"任务难度分布: {dict(zip(*np.unique(list(task_difficulty.values()), return_counts=True)))}")
    return task_difficulty


def extract_geometric_features(row):
    """从数据行中提取几何特征"""
    features = []
    for feature_name in GEOMETRIC_FEATURES:
        value = row.get(feature_name, "0")
        try:
            features.append(float(value))
        except (ValueError, TypeError):
            features.append(0.0)
    return features


def prepare_training_data(rows, task_difficulty):
    """准备训练数据"""
    X = []
    y = []
    task_uids = []

    # 收集所有唯一任务（只取RRTConnect的数据）
    seen_tasks = set()
    for row in rows:
        planner = row.get("planner", "")
        if planner != "RRTConnect":
            continue

        task_uid = row.get("task_uid", "")
        if task_uid in seen_tasks or task_uid not in task_difficulty:
            continue

        seen_tasks.add(task_uid)

        # 提取几何特征
        features = extract_geometric_features(row)
        X.append(features)
        y.append(task_difficulty[task_uid])
        task_uids.append(task_uid)

    return np.array(X), np.array(y), task_uids


def train_classifier(X_train, y_train, X_test, y_test):
    """训练XGBoost分类器"""
    # 标签编码
    label_map = {"easy": 0, "medium": 1, "hard": 2}
    y_train_encoded = np.array([label_map[label] for label in y_train])
    y_test_encoded = np.array([label_map[label] for label in y_test])

    # XGBoost参数
    params = {
        "objective": "multi:softmax",
        "num_class": 3,
        "max_depth": 4,
        "learning_rate": 0.1,
        "n_estimators": 100,
        "random_state": 42,
        "min_child_weight": 3,
    }

    # 训练模型
    model = xgb.XGBClassifier(**params)
    model.fit(X_train, y_train_encoded)

    # 预测
    y_pred = model.predict(X_test)

    # 反向映射标签
    reverse_label_map = {v: k for k, v in label_map.items()}
    y_test_labels = [reverse_label_map[i] for i in y_test_encoded]
    y_pred_labels = [reverse_label_map[i] for i in y_pred]

    return model, y_test_labels, y_pred_labels, label_map


def save_model(model, label_map, output_dir: Path, metadata: dict):
    """保存模型和元数据"""
    output_dir.mkdir(parents=True, exist_ok=True)

    # 保存XGBoost模型
    model_path = output_dir / "difficulty_classifier.json"
    model.save_model(str(model_path))

    # 保存标签映射
    label_map_path = output_dir / "label_map.json"
    with label_map_path.open("w", encoding="utf-8") as f:
        json.dump(label_map, f, indent=2, ensure_ascii=False)

    # 保存元数据
    metadata_path = output_dir / "model_metadata.json"
    with metadata_path.open("w", encoding="utf-8") as f:
        json.dump(metadata, f, indent=2, ensure_ascii=False)

    print(f"\n模型已保存到: {output_dir}")
    print(f"  - 模型文件: {model_path}")
    print(f"  - 标签映射: {label_map_path}")
    print(f"  - 元数据: {metadata_path}")


def main():
    """设置输出目录."""
    parser = argparse.ArgumentParser(description="训练场景难度分类器 V2")
    parser.add_argument(
        "--training-table",
        type=Path,
        default=TRAINING_TABLE_PATH,
        help="训练表路径",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="输出目录（默认使用时间戳）",
    )
    args = parser.parse_args()

    # 设置输出目录
    if args.output_dir is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output_dir = OUTPUT_DIR / timestamp

    print("=" * 60)
    print("场景难度分类器训练 V2")
    print("=" * 60)

    # 加载数据
    print("\n[1/5] 加载训练表数据...")
    rows = load_training_table(args.training_table)

    # 提取难度标签
    print("\n[2/5] 提取RRTConnect baseline难度标签...")
    task_difficulty = extract_task_difficulty_labels(rows)

    # 准备训练数据
    print("\n[3/5] 准备训练数据...")
    X, y, task_uids = prepare_training_data(rows, task_difficulty)
    print(f"特征维度: {X.shape}")
    print(f"样本数量: {len(y)}")
    unique_labels, label_counts = np.unique(y, return_counts=True)
    print(f"难度分布: {dict(zip(unique_labels, label_counts))}")

    # 检查是否有足够的样本
    if len(y) < 10:
        print("\n警告：样本数量太少，无法训练有效模型")
        return

    # 划分训练集和测试集
    X_train, X_test, y_train, y_test, uids_train, uids_test = train_test_split(
        X, y, task_uids, test_size=0.2, random_state=42, stratify=y
    )

    # 训练模型
    print("\n[4/5] 训练XGBoost分类器...")
    model, y_test_labels, y_pred_labels, label_map = train_classifier(
        X_train, y_train, X_test, y_test
    )

    # 评估
    print("\n[5/5] 模型评估:")
    print("\n分类报告:")
    print(classification_report(y_test_labels, y_pred_labels))
    print("\n混淆矩阵:")
    print(confusion_matrix(y_test_labels, y_pred_labels))

    # 特征重要性
    print("\n特征重要性 (Top 10):")
    feature_importance = model.feature_importances_
    feature_names = GEOMETRIC_FEATURES
    importance_pairs = sorted(zip(feature_names, feature_importance), key=lambda x: x[1], reverse=True)
    for name, importance in importance_pairs[:10]:
        print(f"  {name}: {importance:.4f}")

    # 保存模型
    metadata = {
        "training_date": datetime.now().isoformat(),
        "num_samples": int(len(y)),
        "num_features": int(X.shape[1]),
        "train_size": int(len(y_train)),
        "test_size": int(len(y_test)),
        "difficulty_distribution": {str(k): int(v) for k, v in zip(unique_labels, label_counts)},
        "feature_names": GEOMETRIC_FEATURES,
    }
    save_model(model, label_map, args.output_dir, metadata)

    print("\n" + "=" * 60)
    print("训练完成！")
    print("=" * 60)


if __name__ == "__main__":
    main()

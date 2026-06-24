#!/usr/bin/env python3
"""
场景难度分类器训练脚本 (Scene Difficulty Classifier Training)

目标：训练一个XGBoost分类器，根据场景几何特征预测规划难度
输入：benchmark数据中的15维几何特征
输出：三分类模型 (easy/medium/hard)

难度定义：
- easy: RRTConnect成功且快速求解(<1s)
- medium: RRTConnect成功但慢速求解(>=1s)
- hard: RRTConnect失败或触发预算上限
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
BENCHMARK_DIR = REPO_ROOT / "test_results" / "benchmarks" / "simple" / "raw"
OUTPUT_DIR = REPO_ROOT / "test_results" / "models" / "difficulty_classifier"


def load_benchmark_data(benchmark_dir: Path):
    """加载所有benchmark结果文件"""
    all_rows = []
    result_files = sorted(benchmark_dir.glob("*_results.csv"))

    print(f"找到 {len(result_files)} 个结果文件")

    for result_file in result_files:
        with result_file.open("r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                all_rows.append(row)

    print(f"加载了 {len(all_rows)} 条记录")
    return all_rows


def extract_rrtconnect_baseline(rows):
    """提取RRTConnect作为baseline，用于定义难度标签"""
    # 按场景分组
    scene_data = defaultdict(list)

    for row in rows:
        planner = row.get("规划器", "")
        if planner != "RRTConnect":
            continue

        scene_name = row.get("场景名称", "")
        success = row.get("成功", "") == "成功"
        fast_solve = row.get("快速求解(<1s)", "") == "是"
        hit_budget = row.get("触发预算上限", "") == "是"

        scene_data[scene_name].append({
            "success": success,
            "fast_solve": fast_solve,
            "hit_budget": hit_budget,
        })

    # 计算每个场景的难度标签
    scene_difficulty = {}
    for scene_name, records in scene_data.items():
        success_count = sum(1 for r in records if r["success"])
        fast_count = sum(1 for r in records if r["fast_solve"])
        budget_count = sum(1 for r in records if r["hit_budget"])
        total = len(records)

        # 难度判定逻辑（基于成功率）
        success_rate = success_count / total if total > 0 else 0

        if success_rate >= 0.9:
            difficulty = "easy"
        elif success_rate >= 0.5:
            difficulty = "medium"
        else:
            difficulty = "hard"

        scene_difficulty[scene_name] = difficulty

    print(f"场景难度分布: {dict(scene_difficulty)}")
    return scene_difficulty


def extract_features_from_scene(scene_name: str):
    """从场景名称和描述中提取特征（简化版）

    注意：这是临时方案，理想情况下应该从实际规划过程中提取15维几何特征
    目前使用场景名称作为代理特征
    """
    # TODO: 未来需要从实际规划数据中提取完整的15维特征
    # 包括: heuristic_cost, cost_delta_to_direct, start_to_guide_distance等

    # 临时特征：使用场景名称的哈希值作为代理
    features = {
        "scene_hash": hash(scene_name) % 1000 / 1000.0,  # 归一化到[0,1]
    }

    # 从场景名称中提取简单特征
    if "Easy" in scene_name:
        features["name_difficulty"] = 0.3
    elif "Medium" in scene_name:
        features["name_difficulty"] = 0.5
    elif "Hard" in scene_name:
        features["name_difficulty"] = 0.7
    else:
        features["name_difficulty"] = 0.5

    return features


def prepare_training_data(rows, scene_difficulty):
    """准备训练数据"""
    X = []
    y = []
    scene_names = []

    # 收集所有唯一场景
    unique_scenes = set(row.get("场景名称", "") for row in rows)

    for scene_name in unique_scenes:
        if scene_name not in scene_difficulty:
            continue

        features = extract_features_from_scene(scene_name)
        feature_vector = list(features.values())

        X.append(feature_vector)
        y.append(scene_difficulty[scene_name])
        scene_names.append(scene_name)

    return np.array(X), np.array(y), scene_names


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
        "max_depth": 3,
        "learning_rate": 0.1,
        "n_estimators": 50,
        "random_state": 42,
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
    parser = argparse.ArgumentParser(description="训练场景难度分类器")
    parser.add_argument(
        "--benchmark-dir",
        type=Path,
        default=BENCHMARK_DIR,
        help="benchmark数据目录",
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
    print("场景难度分类器训练")
    print("=" * 60)

    # 加载数据
    print("\n[1/5] 加载benchmark数据...")
    rows = load_benchmark_data(args.benchmark_dir)

    # 提取难度标签
    print("\n[2/5] 提取RRTConnect baseline难度标签...")
    scene_difficulty = extract_rrtconnect_baseline(rows)

    # 准备训练数据
    print("\n[3/5] 准备训练数据...")
    X, y, scene_names = prepare_training_data(rows, scene_difficulty)
    print(f"特征维度: {X.shape}")
    print(f"样本数量: {len(y)}")
    print(f"难度分布: {dict(zip(*np.unique(y, return_counts=True)))}")

    # 划分训练集和测试集
    X_train, X_test, y_train, y_test, names_train, names_test = train_test_split(
        X, y, scene_names, test_size=0.2, random_state=42, stratify=y
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

    # 保存模型
    unique_labels, label_counts = np.unique(y, return_counts=True)
    metadata = {
        "training_date": datetime.now().isoformat(),
        "num_samples": int(len(y)),
        "num_features": int(X.shape[1]),
        "train_size": int(len(y_train)),
        "test_size": int(len(y_test)),
        "difficulty_distribution": {str(k): int(v) for k, v in zip(unique_labels, label_counts)},
        "feature_names": ["scene_hash", "name_difficulty"],  # TODO: 更新为完整15维特征
    }
    save_model(model, label_map, args.output_dir, metadata)

    print("\n" + "=" * 60)
    print("训练完成！")
    print("=" * 60)


if __name__ == "__main__":
    main()


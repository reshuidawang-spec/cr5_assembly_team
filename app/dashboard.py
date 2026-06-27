"""数据看板 —— 5号同学实现

展示实验对比图表：总完成时间、机械臂利用率、等待时间、冲突次数。
"""
# TODO: 使用 matplotlib 绘制对比图表
# import matplotlib.pyplot as plt
# from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


def generate_comparison_chart(baseline_data: dict, proposed_data: dict, save_path: str = None):
    """生成 Baseline vs Proposed 对比图"""
    # TODO: 实现图表绘制
    pass


def compute_kpi(tasks: list, robots: list) -> dict:
    """从任务和机械臂状态计算关键指标

    返回: {
        "makespan": float,
        "utilization": dict,  # {robot_id: rate}
        "avg_waiting_time": float,
        "conflict_count": int,
    }
    """
    # TODO: 从日志计算真实 KPI
    return {
        "makespan": 0.0,
        "utilization": {},
        "avg_waiting_time": 0.0,
        "conflict_count": 0,
    }

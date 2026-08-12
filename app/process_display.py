"""Product-aware process labels shared by the execution and analysis tabs."""

PROCESS_LABELS = {
    "box_feed": "箱体上料",
    "pcb_install": "PCB安装",
    "module_install": "模块安装",
    "terminal_install": "端子安装",
    "transfer_to_inspection": "转移检测",
    "screw": "锁付",
    "inspect": "检测",
    "sort_good": "良品分拣",
    "sort_defect": "不良品分拣",
}


def process_label(process: str, product_type: str = "") -> str:
    """Return a concise label that exposes B's integrated route."""
    process = str(process)
    if str(product_type).strip().upper() == "B":
        if process == "module_install":
            return "一体化模块安装"
        if process == "screw":
            return "加强锁付"
    return PROCESS_LABELS.get(process, process)


__all__ = ["PROCESS_LABELS", "process_label"]

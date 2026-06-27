"""CoppeliaSim 通信客户端 —— 2号/3号同学实现

参考 interfaces/sim_interface.py 中的 ISimBridge 接口。
当前直接导入 mock 实现，完成后替换为真实 CoppeliaSim Remote API 调用。
"""
# TODO: 替换为真实 CoppeliaSim 连接
# from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from mock.mock_sim_bridge import MockSimBridge as SimBridge

"""rospy 参数加载辅助。"""

import rospy

from ai_robot_nav.scan_utils import Footprint


def param(name: str, default):
    """读取私有参数 ~name，不存在时返回 default。"""
    return rospy.get_param('~' + name, default)


def param_bool(name: str, default: bool) -> bool:
    value = param(name, default)
    if isinstance(value, str):
        return value.lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def param_float(name: str, default: float) -> float:
    return float(param(name, default))


def param_int(name: str, default: int) -> int:
    return int(param(name, default))


def global_param(name: str, default):
    """读取全局参数（nav_params.yaml 根级键，如 /scan_processing/...）。"""
    return rospy.get_param(name, default)


def global_param_float(name: str, default: float) -> float:
    return float(global_param(name, default))


def load_footprint(margin: float):
    """读取 /robot_footprint；半径未配置或不大于 0 时返回 None，节点退回窄扇区。"""
    radius = global_param_float('/robot_footprint/radius', 0.0)
    if radius <= 0.0:
        return None
    return Footprint(
        radius=radius,
        laser_x=global_param_float('/robot_footprint/laser_x', 0.0),
        margin=max(0.0, margin),
    )

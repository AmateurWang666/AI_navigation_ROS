"""rospy 参数加载辅助。"""

import rospy


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

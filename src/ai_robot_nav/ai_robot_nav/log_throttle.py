"""rospy 日志节流：替代 rclpy 的 throttle_duration_sec。"""

import time


class LogThrottle:
    """按 key 限制同类日志的输出频率。"""

    def __init__(self):
        self._last = {}

    def _should_emit(self, key: str, period: float) -> bool:
        now = time.monotonic()
        last = self._last.get(key)
        if last is None or now - last >= period:
            self._last[key] = now
            return True
        return False

    def warn(self, key: str, msg: str, period: float = 5.0):
        if self._should_emit(key, period):
            import rospy
            rospy.logwarn(msg)

    def error(self, key: str, msg: str, period: float = 5.0):
        if self._should_emit(key, period):
            import rospy
            rospy.logerr(msg)

    def info(self, key: str, msg: str, period: float = 5.0):
        if self._should_emit(key, period):
            import rospy
            rospy.loginfo(msg)

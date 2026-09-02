"""命令行下发目标点，并等待导航结果。

    rosrun ai_robot_nav send_goal 前台         # 按名字
    rosrun ai_robot_nav send_goal 2.5 3.0      # 按坐标
    rosrun ai_robot_nav send_goal 2.5 3.0 1.57 # 按坐标并指定抵达朝向
    rosrun ai_robot_nav send_goal --list       # 列出可用的命名目的地
    rosrun ai_robot_nav send_goal 前台 --no-wait

走 actionlib 而不是直接往 ``/move_base_simple/goal`` 发话题，是为了拿到结果。
话题方式发完就结束，使用者无从知道机器人到底有没有到——是还在路上、被障碍
卡住、还是规划失败早就放弃了。actionlib 会回传终态，因此这个命令可以阻塞到
真正有结论为止，也能作为脚本里的一步来判断成败（退出码非零即未抵达）。

RViz 的 2D Nav Goal 走的仍是 ``/move_base_simple/goal``，两条路径并存不冲突。
"""

import math
import sys
from typing import List

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from ai_robot_nav.goals import Destination, GoalError, format_destinations, \
    parse_destinations, resolve_destination

# actionlib 终态码到中文说明的对应。move_base 失败时只给一个数字，
# 而这几种失败的处理方式完全不同，必须区分开告诉使用者。
TERMINAL_STATES = {
    GoalStatus.SUCCEEDED: '已抵达目标点',
    GoalStatus.ABORTED: '导航失败：move_base 放弃了（通常是规划不出路径，或恢复行为用尽）',
    GoalStatus.REJECTED: '目标点被拒绝：可能在地图之外，或落在障碍物里',
    GoalStatus.PREEMPTED: '导航被中断：有新目标点覆盖了它，或被主动取消',
    GoalStatus.LOST: '目标丢失：move_base 可能中途退出了',
}


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """绕 z 轴的偏航角转四元数。

    只有一个自由度，不必引入 tf 的完整变换：地面移动机器人的目标位姿里
    roll 和 pitch 恒为零。
    """
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def build_goal(destination: Destination, frame_id: str) -> MoveBaseGoal:
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = frame_id
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = destination.x
    goal.target_pose.pose.position.y = destination.y
    goal.target_pose.pose.orientation = yaw_to_quaternion(destination.yaw)
    return goal


def _load_destination_table():
    """读取命名目的地表。

    表放在全局参数 ``/destinations`` 下而不是某个节点的私有参数：它跟着地图走，
    换地图就换一套目的地，由 navigation.launch 随地图一起加载。
    """
    try:
        return parse_destinations(rospy.get_param('/destinations', {}))
    except GoalError as exc:
        rospy.logerr(f'目的地表配置有误: {exc}')
        return {}


def main(argv: List[str] = None):
    argv = list(argv if argv is not None else sys.argv[1:])
    # rosrun 会附加 __name:= / __log:= 这类重映射参数，它们不是目的地参数，先滤掉。
    argv = [arg for arg in argv if not arg.startswith('__')]

    wait = True
    if '--no-wait' in argv:
        argv.remove('--no-wait')
        wait = False

    rospy.init_node('send_goal', anonymous=True)
    table = _load_destination_table()

    if '--list' in argv or '-l' in argv:
        print('可用的命名目的地:')
        print(format_destinations(table))
        return 0

    try:
        destination, source = resolve_destination(argv, table)
    except GoalError as exc:
        print(f'错误: {exc}', file=sys.stderr)
        return 2

    frame_id = rospy.get_param('~goal_frame', 'map')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    print(f'等待 move_base ...')
    if not client.wait_for_server(rospy.Duration(10.0)):
        print('错误: 10 秒内没等到 move_base。导航栈没启动？\n'
              '  先运行: bash scripts/sim.sh nav', file=sys.stderr)
        return 1

    print(f'目标点来自{source}: '
          f'x={destination.x:.2f} y={destination.y:.2f} '
          f'yaw={destination.yaw:.2f} (坐标系 {frame_id})')
    client.send_goal(build_goal(destination, frame_id))

    if not wait:
        print('已下发（--no-wait，不等待结果）。')
        return 0

    print('导航中... (Ctrl+C 取消)')
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        # 直接退出会把机器人留在半路上继续执行旧目标，必须显式撤销。
        client.cancel_goal()
        print('\n已取消目标点。')
        return 130

    state = client.get_state()
    message = TERMINAL_STATES.get(state, f'导航结束，未知状态码 {state}')
    print(message)
    return 0 if state == GoalStatus.SUCCEEDED else 1


if __name__ == '__main__':
    sys.exit(main())

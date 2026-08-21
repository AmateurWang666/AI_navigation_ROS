"""进程生命周期相关的共用工具。

两个节点的退出契约是一样的：必须活到把一条零速度指令发出去之后才能退出。
底盘控制器会锁存最后收到的速度，进程直接消失等于让机器人带着最后一条指令
一直跑下去。
"""

import signal


def install_shutdown_signals():
    """把 SIGINT 和 SIGTERM 转成主线程里的 KeyboardInterrupt。

    不能依赖继承来的信号处置方式。用 shell 把节点作为后台任务启动时，SIGINT
    会被置为 SIG_IGN；若不显式安装处理器，进程可能完全忽略 Ctrl-C。
    SIGTERM 同样要接：launch 在节点迟迟不退出时会升级到 SIGTERM。
    """
    def _interrupt(signum, _frame):
        # 先把处置恢复成默认：万一清理流程自己卡住了（比如卡在一个未超时的
        # HTTP 请求里），第二次信号能直接强杀进程，而不是又被同一个处理器吞掉。
        signal.signal(signum, signal.SIG_DFL)
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _interrupt)
    signal.signal(signal.SIGTERM, _interrupt)

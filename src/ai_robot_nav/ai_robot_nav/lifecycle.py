"""Shared process lifecycle helpers.

Both nodes need the same shutdown contract: stay alive long enough to push a
zero velocity out, because the base controller latches whatever it received
last. That rules out rclpy's built-in signal handling, which tears the context
down before a node's cleanup can publish anything.
"""

import signal


def install_shutdown_signals():
    """Turn SIGINT and SIGTERM into KeyboardInterrupt in the main thread.

    The inherited disposition cannot be relied on. A shell that starts the node
    as a background job leaves SIGINT set to SIG_IGN, so with rclpy's handlers
    disabled the process would ignore Ctrl-C entirely. SIGTERM matters too:
    launch escalates to it when a node is slow to stop, and Python would
    otherwise treat it as a silent kill with no chance to brake.
    """
    def _interrupt(signum, _frame):
        # Restore the default so a second signal force-kills a wedged shutdown
        # instead of being swallowed by cleanup that is already stuck.
        signal.signal(signum, signal.SIG_DFL)
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _interrupt)
    signal.signal(signal.SIGTERM, _interrupt)

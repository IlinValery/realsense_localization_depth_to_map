import threading


class StoppableThread(threading.Thread):
    """Thread class with a stop() method. The thread itself has to check
    regularly for the is_executed() condition."""

    def __init__(self,  *args, **kwargs):
        super(StoppableThread, self).__init__(*args, **kwargs)
        self._execution_status = True
#         self._stop_event = threading.Event()

    def stop(self):
        self._execution_status = False

    def is_executed(self):
        return self._execution_status
